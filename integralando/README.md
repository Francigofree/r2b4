# R2B4 P0 dual-frame ER2 navigation test hotfix

Base source:
- repository: Francigofree/r2b4
- commit: c020bfa712a489aaf72c69e7bcf9920f2f8d87b1
- guarded file blob: tests/feature/test_er2_navigation_motion.py = eb03b03a9e25b3356c975c873a126828969c901b

Purpose:
The production GLOBAL_TRANSFORM_MISSING fail-closed behavior remains unchanged.
Only the two legacy ER2 navigation test fixtures are migrated to the current
dual-frame RobotEstimate contract using an identity map->odom transform.

Changes:
- import GLOBAL_FRAME_ID / LOCAL_FRAME_ID / Pose2D
- add _dual_frame_estimate(...) synthetic downstream-of-L3 fixture
- local world/costmap use R2B4_ODOM_LOCAL
- global NAVIGATE tests use complete global/local/map_to_odom estimates
- moved estimate in the async test updates both local and global pose consistently

Apply:
    python3 apply_hotfix.py --check /home/alba/project_r2b4
    python3 apply_hotfix.py /home/alba/project_r2b4

Verify:
    bash verify_hotfix.sh /home/alba/project_r2b4
    bash verify_hotfix.sh /home/alba/project_r2b4 --full
