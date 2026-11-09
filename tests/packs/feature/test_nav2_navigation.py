"""Nonblocking Nav2 velocity transport and original-evidence expiry."""
from dataclasses import replace
import math
import socket
import time

import pytest

from v3.adapters.nav2_navigation import (Nav2NavigationClient, Nav2NavigationConfig,
    decode_packet, encode_packet, request_packet)
from v3.contracts import CommandMode, GLOBAL_FRAME_ID, LOCAL_FRAME_ID, TickContext, Waypoint
from v3.contracts.navigation import Nav2NavigationRequest, Nav2NavigationStatus
from v3.nav2_navigation_bridge import validate_request, velocity_lifetime
from v3.slam_toolbox_bridge import odometry_twist
from v3.adapters.slam_toolbox_transport import SlamOdometryPacket


def request(now=1_000_000_000, **changes):
    value = Nav2NavigationRequest(TickContext(10, now), "mission", "goal", CommandMode.NAVIGATE,
        Waypoint(1., 0.), .3, 1.4, 2, 5)
    return replace(value, **changes)


def reply_for(value, *, sequence=1, generation=100, **changes):
    result = dict(mission_id=value.mission_id, goal_id=value.goal_id,
        source_ns=value.context.monotonic_ns, source_tick_id=value.context.tick_id,
        frame_id=LOCAL_FRAME_ID, odom_generation=value.odom_generation,
        transform_revision=value.transform_revision, status="ACTIVE", worker_generation=generation,
        command_sequence=sequence, produced_ns=value.context.monotonic_ns+10_000_000,
        valid_until_ns=value.context.monotonic_ns+150_000_000,
        scan_measurement_ns=value.context.monotonic_ns-10_000_000,
        odom_measurement_ns=value.context.monotonic_ns-5_000_000,
        v_mps=.2, omega_rad_s=0., selected_goal=[1., 0., None], map_revision=1, child_goal_id=1)
    return {**result, **changes}


@pytest.fixture
def channel(tmp_path):
    path = str(tmp_path/"nav2.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.settimeout(.2)
    server.bind(path)
    client = Nav2NavigationClient(Nav2NavigationConfig(path))
    try:
        yield client, server
    finally:
        client.close()
        server.close()


def send_reply(client, server, value, **changes):
    client.submit(value)
    packet, address = server.recvfrom(4096)
    assert decode_packet(packet) == request_packet(value)
    server.sendto(encode_packet(reply_for(value, **changes)), address)
    return address


def test_latest_guidance_preserves_production_and_measurement_time(channel):
    client, server = channel
    value = request()
    send_reply(client, server, value)
    result = client.poll(TickContext(11, value.context.monotonic_ns+20_000_000))
    assert result.guidance.status is Nav2NavigationStatus.ACTIVE
    assert result.guidance.source_context == value.context
    assert result.guidance.scan_measurement_ns == value.context.monotonic_ns-10_000_000
    assert result.guidance.valid_until_ns == value.context.monotonic_ns+150_000_000


@pytest.mark.parametrize("mutation", [dict(frame_id=GLOBAL_FRAME_ID), dict(v_mps=math.nan),
    dict(mission_id="old"), dict(odom_generation=1), dict(source_tick_id=11),
    dict(valid_until_ns=2_000_000_000)])
def test_invalid_future_or_superseded_result_cannot_supply_motion(channel, mutation):
    client, server = channel
    value = request()
    client.submit(value)
    _, address = server.recvfrom(4096)
    # JSON itself must allow NaN here to exercise the receiver validation.
    import json
    server.sendto(json.dumps(reply_for(value, **mutation)).encode(), address)
    assert client.poll(TickContext(11, value.context.monotonic_ns+20_000_000)).guidance is None


def test_generation_sequence_highwater_survives_invalid_packet(channel):
    client, server = channel
    value = request()
    address = send_reply(client, server, value, generation=200, sequence=10)
    context = TickContext(11, value.context.monotonic_ns+20_000_000)
    assert client.poll(context).guidance.worker_generation == 200
    server.sendto(b"{}", address)
    assert client.poll(context).error is not None
    server.sendto(encode_packet(reply_for(value, generation=100, sequence=100)), address)
    assert client.poll(context).guidance is None


def test_transport_failure_clears_previous_guidance_and_never_waits(channel):
    client, server = channel
    value = request()
    send_reply(client, server, value)
    context = TickContext(11, value.context.monotonic_ns+20_000_000)
    assert client.poll(context).guidance is not None
    server.close()
    start = time.monotonic()
    client.submit(replace(value, context=TickContext(12, value.context.monotonic_ns+60_000_000)))
    result = client.poll(TickContext(13, value.context.monotonic_ns+70_000_000))
    assert result.error and result.guidance is None
    assert time.monotonic()-start < .05


def test_mission_change_and_cancellation_clear_cached_velocity(channel):
    client, server = channel
    value = request()
    send_reply(client, server, value)
    context = TickContext(11, value.context.monotonic_ns+20_000_000)
    assert client.poll(context).guidance is not None
    client.submit(replace(value, mission_id="new"))
    assert client.poll(context).guidance is None
    server.recvfrom(4096)
    client.submit(None)
    packet, _ = server.recvfrom(4096)
    assert decode_packet(packet) == dict(operation="CANCEL", mission_id="new", goal_id="goal")


def test_tighter_mission_tolerance_is_sent_immediately_and_invalidates_old_velocity(channel):
    client, server = channel
    value = request()
    send_reply(client, server, value)
    context = TickContext(11, value.context.monotonic_ns+20_000_000)
    assert client.poll(context).guidance is not None
    tighter = replace(value, context=TickContext(11, value.context.monotonic_ns+20_000_000),
                      goal_tolerance_m=.02, yaw_tolerance_rad=.03)
    client.submit(tighter)
    packet, _ = server.recvfrom(4096)
    assert decode_packet(packet)["goal_tolerance_m"] == .02
    assert decode_packet(packet)["yaw_tolerance_rad"] == .03
    assert client.poll(TickContext(12, value.context.monotonic_ns+40_000_000)).guidance is None


@pytest.mark.parametrize("limits", [dict(max_v_mps=.25), dict(max_omega_rad_s=1.)])
def test_mission_cap_change_sends_immediately_and_invalidates_old_velocity(channel, limits):
    client, server = channel
    value = request()
    send_reply(client, server, value)
    assert client.poll(TickContext(11, value.context.monotonic_ns+20_000_000)).guidance is not None
    changed = replace(value, context=TickContext(11, value.context.monotonic_ns+20_000_000), **limits)
    client.submit(changed)
    packet, _ = server.recvfrom(4096)
    for key, limit in limits.items():
        assert decode_packet(packet)[key] == limit
    assert client.poll(TickContext(12, value.context.monotonic_ns+40_000_000)).guidance is None


def test_bridge_rejects_expired_invalid_mission_and_goal_packets():
    value = request()
    packet = request_packet(value)
    validate_request(packet, value.context.monotonic_ns)
    for change in (dict(source_ns=0), dict(frame_id=LOCAL_FRAME_ID), dict(goal=[math.inf, 0., None]),
                   dict(max_v_mps=True), dict(mode="TELEOP"), dict(odom_generation=-1),
                   dict(goal_tolerance_m=math.nan), dict(yaw_tolerance_rad=-.01)):
        with pytest.raises(ValueError):
            validate_request({**packet, **change}, value.context.monotonic_ns)


def test_velocity_ttl_cannot_renew_delayed_sensor_or_previous_goal():
    packet = request_packet(request())
    args = dict(now_ns=1_050_000_000, produced_ns=1_040_000_000,
                accepted_ns=1_010_000_000, scan_ns=950_000_000,
                odom_ns=1_030_000_000, tf_ns=1_020_000_000)
    assert velocity_lifetime(packet, **args) == 1_200_000_000
    for changes in (dict(scan_ns=750_000_000), dict(odom_ns=1_060_000_000),
                    dict(accepted_ns=1_050_000_000), dict(produced_ns=1_060_000_000),
                    dict(tf_ns=0), dict(now_ns=1_400_000_000)):
        assert velocity_lifetime(packet, **{**args, **changes}) is None


def test_slam_odom_twist_uses_measurement_delta_and_same_generation():
    previous = SlamOdometryPacket(1, 2, 3, 1_000_000_000, 4, 0., 0., math.pi/2, True, True)
    current = replace(previous, monotonic_ns=1_050_000_000, tick_id=4, y_m=.01)
    assert odometry_twist(previous, current) == pytest.approx((.2, 0.))
    assert odometry_twist(None, current) is None
    for changes in (dict(generation=5), dict(local_translation_valid=False),
                    dict(monotonic_ns=1_200_000_000)):
        assert odometry_twist(previous, replace(current, **changes)) is None
