"""Human teaching names a stable person without inventing visual re-identification."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from r2b4_orchestration.behavior_system import BehaviorLifecycle, BehaviorSystem
from r2b4_orchestration.person_identity import PersonIdentity, PersonTeaching
from r2b4_orchestration.search_person import SearchPerson
from r2b4_orchestration.semantic_projector import SemanticProjector
from r2b4_orchestration.world_model import PublicWorldModel, ValidityScope, WorldQuery


class Clock:
    now = 10_000_000_000

    def __call__(self):
        return self.now


def setup_identity():
    clock = Clock()
    world = PublicWorldModel(clock_ns=clock, clock_epoch="teaching-boot")
    identity = PersonIdentity(world, clock_ns=clock)
    projector = SemanticProjector(world, clock_ns=clock, person_identity=identity)
    runtime = {"runtime_running": True, "runtime_pid": 123, "capture_mode": "full", "capture_hz": 10}
    vision = {"running": True, "detector_running": True, "owner_generation": "camera-a",
              "owner_pid": 456, "last_error": None, "detector_last_error": None}
    status = {"state": "RUNNING", "tick_id": 10, "monotonic_ns": clock.now,
              "estimate": {"localization_quality": {"generation": 1}},
              "world": {"frame_id": "R2B4_ODOM_LOCAL", "person_tracks": [track(clock)]}}
    projector.completed_status(status, runtime_pid=123, vision_status=vision)
    return clock, world, identity, projector, runtime, status, vision


def track(clock, name="person-7"):
    return {"track_id": name, "x_m": 4.0, "y_m": 3.0, "confidence": .9,
            "measurement_monotonic_ns": clock.now, "prediction_valid_until_ns": clock.now + 500_000_000,
            "estimate_status": "OBSERVED"}


def teach(identity, runtime, status, vision, **parameters):
    return identity.teach(PersonTeaching("Anna", "human-request-1", aliases=("Ani",), **parameters),
                          runtime=runtime, status=status, vision_status=vision)


class SearchRobot:
    def __init__(self, clock, world, runtime, status, vision):
        self.clock, self.world, self.runtime, self.status, self.vision = clock, world, runtime, status, vision
        self.actions = []

    def read(self, resource):
        return {"operator.status": self.runtime, "v3.status": self.status, "camera.status": self.vision,
                "world.snapshot": self.world.snapshot().to_jsonable()}[resource]

    def query(self, query):
        return self.world.query(query)

    def execute(self, action, **parameters):
        self.actions.append((action, parameters))
        raise AssertionError("fresh taught person requires no search motion")

    def stop(self):
        pass


def search(clock, world, runtime, status, vision, entity, **parameters):
    robot = SearchRobot(clock, world, runtime, status, vision)
    system = BehaviorSystem(robot, clock_ns=clock)
    system.register("search_person", SearchPerson)
    return robot, system.start("search_person", {"entity_id": entity, **parameters})


def test_explicit_teaching_publishes_stable_identity_and_exact_named_follow_evidence():
    clock, world, identity, _, runtime, status, vision = setup_identity()
    anonymous = world.read("person:123:person-7", "location").observation
    taught = teach(identity, runtime, status, vision)
    entity = taught["entity_id"]
    assert entity != anonymous.entity_id
    assert identity.resolve("ANI") == entity
    stable = world.read(entity, "identity")
    assert stable.domain == "person_identity" and stable.value["name"] == "Anna"
    assert stable.observation.validity_scope is None
    assert world.read("person:123:person-7", "location").observation == anonymous
    target = identity.validate_target(taught, runtime=runtime, status=status, vision_status=vision)
    assert target["target_entity_id"] == entity and target["target_track_id"] == "person-7"
    assert target["vision_generation"] == "camera-a" and target["vision_owner_pid"] == 456
    assert target["localization_generation"] == 1 and target["runtime_pid"] == 123
    robot, result = search(clock, world, runtime, status, vision, entity, require_bound_track=True)
    assert result.lifecycle is BehaviorLifecycle.COMPLETED and robot.actions == []
    assert dict(result.result)["bound_track"] is True
    assert dict(result.result)["observation_qualified"] is True


def test_multiple_people_need_explicit_track_selection_and_cannot_share_one_binding():
    clock, world, identity, _, runtime, status, vision = setup_identity()
    status["world"]["person_tracks"].append(track(clock, "person-8"))
    with pytest.raises(ValueError, match="CLARIFICATION"):
        teach(identity, runtime, status, vision)
    assert world.query(WorldQuery(domain="person_identity")).facts == ()
    named = teach(identity, runtime, status, vision, target_track_id="person-8")
    assert named["target_track_id"] == "person-8"
    with pytest.raises(ValueError, match="TRACK_ALREADY_BOUND"):
        identity.teach(PersonTeaching("Bela", "human-request-2", target_track_id="person-8"),
                       runtime=runtime, status=status, vision_status=vision)
    assert identity.resolve("Bela") is None


@pytest.mark.parametrize("change", ["old_status", "old_track", "predicted", "no_generation", "camera_failed", "no_local_generation"])
def test_unqualified_context_cannot_teach_name_or_binding(change):
    clock, world, identity, _, runtime, status, vision = setup_identity()
    if change == "old_status":
        clock.now += 3_000_000_000
    elif change == "old_track":
        status["world"]["person_tracks"][0]["prediction_valid_until_ns"] = clock.now - 1
    elif change == "predicted":
        status["world"]["person_tracks"][0]["estimate_status"] = "PREDICTED"
    elif change == "no_generation":
        vision.pop("owner_generation")
    elif change == "camera_failed":
        vision["last_error"] = "camera failed"
    else:
        status["estimate"]["localization_quality"].pop("generation")
    with pytest.raises(ValueError):
        teach(identity, runtime, status, vision)
    assert identity.resolve("Anna") is None and identity.has_bindings is False


def test_same_status_poll_cannot_refresh_original_measurement_or_extend_expiry():
    clock, world, identity, projector, runtime, status, vision = setup_identity()
    taught = teach(identity, runtime, status, vision)
    scope = ValidityScope(frame_id="R2B4_ODOM_LOCAL", runtime_pid=123)
    original = world.query(WorldQuery(entity_id=taught["entity_id"], attribute="person_binding", scope=scope)).facts[0]
    clock.now += 100_000_000
    projector.completed_status(status, runtime_pid=123, vision_status=vision)
    same = world.query(WorldQuery(entity_id=taught["entity_id"], attribute="person_binding", scope=scope)).facts[0]
    assert same.observation == original.observation
    clock.now += 100_000_000
    status.update(tick_id=11, monotonic_ns=clock.now)
    status["world"]["person_tracks"] = [track(clock)]
    projector.completed_status(status, runtime_pid=123, vision_status=vision)
    updated = identity.validate_target(taught, runtime=runtime, status=status, vision_status=vision)
    assert updated["measurement_time_ns"] == clock.now
    assert updated["expires_ns"] > original.value["expires_ns"]


@pytest.mark.parametrize("change", ["track_lost", "vision_generation", "vision_owner", "local_generation", "runtime", "vision_unavailable", "expiry"])
def test_lost_source_identity_never_automatically_reacquires_same_numbered_track(change):
    clock, world, identity, projector, runtime, status, vision = setup_identity()
    taught = teach(identity, runtime, status, vision)
    good_status, good_vision = deepcopy(status), deepcopy(vision)
    if change == "track_lost":
        status["world"]["person_tracks"] = []
    elif change == "vision_generation":
        vision["owner_generation"] = "camera-b"
    elif change == "vision_owner":
        vision["owner_pid"] = 457
    elif change == "local_generation":
        status["estimate"]["localization_quality"]["generation"] = 2
    elif change == "runtime":
        runtime["runtime_pid"] = 124
    elif change == "vision_unavailable":
        vision["detector_running"] = False
    else:
        clock.now += 600_000_000
        status["monotonic_ns"] = clock.now
        status["world"]["person_tracks"] = [track(clock)]
    projector.completed_status(status, runtime_pid=runtime["runtime_pid"], vision_status=vision)
    assert identity.has_bindings is False
    assert identity.resolve("Anna") == taught["entity_id"]
    status, vision = good_status, good_vision
    clock.now += 10_000_000
    status.update(monotonic_ns=clock.now, tick_id=12)
    status["world"]["person_tracks"] = [track(clock)]
    runtime["runtime_pid"] = 123
    projector.completed_status(status, runtime_pid=123, vision_status=vision)
    assert identity.has_bindings is False
    with pytest.raises(ValueError, match="NEW_TEACHING"):
        identity.validate_target(taught, runtime=runtime, status=status, vision_status=vision)


def test_restart_keeps_name_but_requires_new_explicit_binding_even_in_same_boot_and_pid():
    clock, world, identity, _, runtime, status, vision = setup_identity()
    taught = teach(identity, runtime, status, vision)
    restored = PublicWorldModel(clock_ns=clock, clock_epoch="teaching-boot")
    restored.restore(world.export_state())
    new_identity = PersonIdentity(restored, clock_ns=clock)
    assert new_identity.resolve("Anna") == taught["entity_id"]
    with pytest.raises(ValueError, match="NEW_TEACHING"):
        new_identity.validate_target(taught, runtime=runtime, status=status, vision_status=vision)
    assert new_identity.has_bindings is False


def test_trackless_or_unverified_external_named_fact_is_only_information():
    clock, world, _, _, runtime, status, vision = setup_identity()
    world.observe("person:anna", "location", {"place_id": "room:lounge", "runtime_pid": 123,
                  "target_track_id": "person-7"}, domain="person_position", measurement_time_ns=clock.now,
                  confidence=.9, source="semantic_vision", lineage=("external:4",))
    robot, result = search(clock, world, runtime, status, vision, "person:anna")
    assert result.lifecycle is BehaviorLifecycle.COMPLETED and robot.actions == []
    info = dict(result.result)
    assert info["semantic_evidence"] is True and info["follow_target_available"] is False
    assert "target_track_id" not in info
    robot, result = search(clock, world, runtime, status, vision, "person:anna", require_bound_track=True)
    assert result.lifecycle is BehaviorLifecycle.FAILED and robot.actions == []


def test_agent_claim_and_anonymous_entity_renaming_cannot_create_named_identity():
    _, world, identity, _, runtime, status, vision = setup_identity()
    with pytest.raises(ValueError, match="HUMAN_SOURCE"):
        PersonTeaching("Anna", "agent-1", source="AGENT")
    with pytest.raises(ValueError, match="stable person"):
        teach(identity, runtime, status, vision, entity_id="person:123:person-7")
    assert world.query(WorldQuery(domain="person_identity")).facts == ()


def test_duplicate_teaching_is_idempotent_and_cannot_revive_lost_or_restored_binding():
    clock, world, identity, _, runtime, status, vision = setup_identity()
    taught = teach(identity, runtime, status, vision)
    original = world.read(taught["entity_id"], "identity").observation
    duplicate = teach(identity, runtime, status, vision)
    assert duplicate["duplicate"] is True and world.read(taught["entity_id"], "identity").observation == original
    identity.invalidate("TEST_TRACK_LOSS")
    duplicate = teach(identity, runtime, status, vision)
    assert duplicate["follow_target_available"] is False and "target_track_id" not in duplicate
    restored = PublicWorldModel(clock_ns=clock, clock_epoch="teaching-boot")
    restored.restore(world.export_state())
    identity = PersonIdentity(restored, clock_ns=clock)
    clock.now += 10_000_000
    status.update(monotonic_ns=clock.now, tick_id=11)
    status["world"]["person_tracks"] = [track(clock)]
    duplicate = teach(identity, runtime, status, vision)
    assert duplicate["follow_target_available"] is False and identity.has_bindings is False
    rebound = identity.teach(PersonTeaching("Anna", "new-human-confirmation"), runtime=runtime,
                             status=status, vision_status=vision)
    assert rebound["entity_id"] == taught["entity_id"] and rebound["follow_target_available"] is True
    assert restored.read(taught["entity_id"], "identity").value["first_teaching_time_ns"] == original.measurement_time_ns


def test_rejected_named_location_publication_never_returns_usable_follow_binding():
    _, world, identity, _, runtime, status, vision = setup_identity()
    observe = world.observe

    def reject_location(entity, attribute, value, **parameters):
        if attribute == "location" and parameters.get("source") == "person_identity:HUMAN":
            return SimpleNamespace(accepted=False, reason="OUT_OF_ORDER")
        return observe(entity, attribute, value, **parameters)

    world.observe = reject_location
    with pytest.raises(ValueError, match="PUBLICATION_NOT_ACCEPTED"):
        teach(identity, runtime, status, vision)
    assert identity.has_bindings is False
    entity = identity.resolve("Anna")
    assert entity is not None  # The explicit durable name is still truthful.
    assert world.read(entity, "person_binding").value["active"] is False
