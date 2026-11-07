"""Production scan feedback keeps SLAM deskew in continuous odometry."""
from dataclasses import replace

import pytest

from rig import healthy_localization, resolved_config
from test_v3_bno055_imu_backend import Device
from test_v3_latest_lidar_backend import Port
from v3_test_fixtures import FakeGpio
from v3.adapters.native_lidar_port import TimedPoseReference
from v3.adapters.process_vision_port import UnavailableVisionPort
from v3.contracts import (
    FinalActuation, GLOBAL_FRAME_ID, LOCAL_FRAME_ID, Pose2D, RobotEstimate,
    SafetyDecision, TickContext,
)
from v3.engine import LayerRecord, TickResult, TickTrace
from v3_hardware_runtime import NativeHardwareSensorOwner


@pytest.mark.parametrize("backend", ("native", "slam_toolbox"))
def test_hardware_owner_selects_scan_feedback_frame_without_motor_output(backend):
    resolved = resolved_config()

    class Lidar(Port):
        config = replace(resolved.lidar, global_backend=backend)

        def __init__(self):
            super().__init__()
            self.poses = []
            self.slam_odometry = []

        def publish_pose_reference(self, pose):
            self.poses.append(pose)

        def publish_slam_odometry(self, estimate):
            self.slam_odometry.append(estimate)

    lidar = Lidar()
    owner = NativeHardwareSensorOwner(
        FakeGpio(), lambda bus: None, lambda pose: lidar,
        replace(resolved.runtime.sensor_inputs, person_photo_evidence=None),
        open_imu_device=lambda config: Device({}),
        open_vision_port=lambda *args, **kwargs: UnavailableVisionPort("offline test"),
    )
    try:
        for tick, (x, y, yaw) in enumerate(((.8, -.4, .2), (1.2, -.7, .45))):
            context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
            global_pose = Pose2D(GLOBAL_FRAME_ID, x, y, yaw)
            estimate = RobotEstimate(
                context, GLOBAL_FRAME_ID, x, y, yaw, 0., 0., (0.,) * 25,
                local_pose=Pose2D(LOCAL_FRAME_ID, 0., 0., 0.),
                global_pose=global_pose, map_to_odom=global_pose,
                localization_quality=healthy_localization(),
            )
            final = FinalActuation(context, 0., 0., False, SafetyDecision.STOP,
                                   "CLEAR", "OFFLINE_TEST")
            owner.publish_tick_result(TickResult(
                final, TickTrace(context, (LayerRecord("L3", estimate),))))
            expected = (0., 0., 0.) if backend == "slam_toolbox" else (x, y, yaw)
            assert lidar.poses[-1] == TimedPoseReference(context.monotonic_ns, *expected)
            assert lidar.slam_odometry[-1] is estimate
        assert len(lidar.poses) == 2
    finally:
        owner.close()
    assert lidar.stop_calls == 1
