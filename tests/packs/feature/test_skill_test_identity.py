"""A successful test belongs to both the skill and the executed test source."""
from __future__ import annotations

import hashlib
import os
from pathlib import Path

from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolSpec
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.skill_learning import SkillLearningLoop
from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.software_interface import test_skill as execute_skill_test


SKILL_SOURCE = "async def run(robot):\n    return {'count': 2}\n"
PASSING_TEST = '''import asyncio
from changeable_assertions import run

def test_returned_count():
    assert asyncio.run(run(None))["count"] == 2
'''
FAILING_TEST = PASSING_TEST.replace('== 2', '== 3')


def test_changed_saved_test_retests_before_trial_even_when_skill_source_is_unchanged(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    agent_tests, outer_tests, trials = [], [], []
    original_identities = {}

    class Provider:
        model = "scripted-provider"

        def complete_agent_step(self, *_args, **_kwargs):
            return AgentModelReply(self.model, skill_proposal={"kind": "create_skill",
                "name": "changeable_assertions", "source": SKILL_SOURCE, "test_source": PASSING_TEST})

    def agent_test(arguments):
        result = execute_skill_test(library, **arguments)
        agent_tests.append(result)
        return result

    broker = AgentToolBroker((
        (AgentToolSpec("skill.create", "Save skill", "READ"), lambda args: library.create(**args)),
        (AgentToolSpec("skill.test", "Run saved test", "READ"), agent_test),
    ))

    def change_after_pass(event, fields):
        if event != "agent_skill_tested" or fields["status"] != "PASSED":
            return
        loaded = library.load(fields["name"])
        test_path = Path(loaded.descriptor["test_path"])
        old_stat = test_path.stat()
        original_identities.update(source_hash=loaded.source_hash,
                                   test_source_hash=loaded.descriptor["test_source_hash"])
        library.update(loaded.name, loaded.source, test_source=FAILING_TEST)
        # Exercise timestamp/size-based bytecode ambiguity deterministically.
        os.utime(test_path, ns=(old_stat.st_atime_ns, old_stat.st_mtime_ns))

    def outer_test(name, **control):
        result = execute_skill_test(library, name, **control)
        outer_tests.append(result)
        return result

    def trial(*args, **kwargs):
        trials.append((args, kwargs))
        raise AssertionError("a saved failing test must prevent the trial")

    loop = SkillLearningLoop(AgentCore(Provider(), broker), library, trial, test_runner=outer_test)
    summary = loop.run("Return the measured count", max_iterations=1, event_sink=change_after_pass)
    current = library.load("changeable_assertions")
    assert current.source_hash == original_identities["source_hash"] == summary["current_hash"]
    assert current.descriptor["test_source_hash"] != original_identities["test_source_hash"]
    assert len(agent_tests) == len(outer_tests) == 1
    assert agent_tests[0]["status"] == "PASSED" and agent_tests[0]["returncode"] == 0
    assert agent_tests[0]["test_source_hash"] == original_identities["test_source_hash"]
    assert outer_tests[0]["status"] == "FAILED" and outer_tests[0]["returncode"] != 0
    assert outer_tests[0]["test_source_hash"] == current.descriptor["test_source_hash"]
    assert summary["iterations"][0]["tests"] == outer_tests[0]
    assert summary["iterations"][0]["trial"] is None and not trials
    assert summary["status"] != "LEARNED" and summary["goal_satisfaction"] == "UNASSESSED"


def test_passing_pytest_that_rewrites_its_own_source_is_failed_evidence(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    replacement = "def test_replaced_expectation():\n    assert False\n"
    test_source = f'''from pathlib import Path

def test_changes_its_saved_expectation():
    Path(__file__).write_text({replacement!r}, encoding="utf-8")
'''
    saved = library.create("self_editing_test", SKILL_SOURCE, test_source=test_source)
    initial_hash = hashlib.sha256(test_source.encode("utf-8")).hexdigest()
    result = execute_skill_test(library, "self_editing_test", timeout_s=10)
    current = library.load("self_editing_test")
    assert result["returncode"] == 0 and result["timeout"] is False
    assert "1 passed" in result["output"]
    assert result["status"] == "FAILED" and result["test_source_changed"] is True
    assert result["test_source_hash"] == saved["test_source_hash"] == initial_hash
    assert current.descriptor["test_source_hash"] == hashlib.sha256(replacement.encode("utf-8")).hexdigest()
    assert current.descriptor["test_source_hash"] != result["test_source_hash"]
    assert current.source_hash == saved["source_hash"] == result["source_hash"]
    assert result["source_changed"] is False
