import unittest
from types import SimpleNamespace

from core.executor import TaskExecutor
from core.task_model import RobotTask, TaskType
from state import RobotState


class _EncoderService:
    def __init__(self, left=1.0, right=2.0):
        self.snapshot = SimpleNamespace(
            left_distance=float(left),
            right_distance=float(right),
            health="OK",
        )

    def get_snapshot(self):
        return self.snapshot


class _StateMachine:
    def __init__(self):
        self.current_enum = RobotState.IDLE

    def transition_to(self, state, **_kwargs):
        self.current_enum = state


class _Logger:
    def warn(self, *_args, **_kwargs):
        pass


class TaskExecutorEncoderSourceTests(unittest.TestCase):
    def _controller(self, service):
        return SimpleNamespace(
            encoder_service=service,
            # A conflicting compatibility estimator must be ignored.
            estim=SimpleNamespace(
                left=SimpleNamespace(distance=99.0),
                right=SimpleNamespace(distance=99.0),
            ),
            sm=_StateMachine(),
            logger=_Logger(),
            set_speed_level=lambda *_args, **_kwargs: True,
            dock_active=False,
            dock_speed_level=1,
            dock_dir=1,
        )

    def test_move_uses_encoder_service_snapshot_only(self):
        service = _EncoderService()
        ctrl = self._controller(service)
        executor = TaskExecutor(ctrl)
        task = RobotTask(
            type=TaskType.MOVE,
            params={"distance": 0.5, "direction": 1, "speed_level": 3},
        )

        executor.start_task(task)
        self.assertEqual((executor.start_dist_l, executor.start_dist_r), (1.0, 2.0))
        self.assertTrue(executor.is_running)

        service.snapshot.left_distance = 1.6
        service.snapshot.right_distance = 2.6
        executor.tick()

        self.assertFalse(executor.is_running)
        self.assertEqual(ctrl.sm.current_enum, RobotState.IDLE)

    def test_move_is_blocked_without_encoder_snapshot(self):
        service = _EncoderService()
        service.snapshot = None
        ctrl = self._controller(service)
        executor = TaskExecutor(ctrl)

        executor.start_task(
            RobotTask(type=TaskType.MOVE, params={"distance": 0.5, "direction": 1})
        )

        self.assertFalse(executor.is_running)
        self.assertIsNone(executor.current_task)
        self.assertEqual(ctrl.sm.current_enum, RobotState.IDLE)


if __name__ == "__main__":
    unittest.main()
