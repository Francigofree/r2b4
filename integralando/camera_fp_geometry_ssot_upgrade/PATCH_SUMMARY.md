# Patch summary

- `conf/vezerles.json`: removes the legacy live camera FOV/yaw values.
- `v3/config.py`: derives compatibility-only L4 fallback values from `hardver.camera.geometry`; person tracking requires geometry.
- `v3/adapters/person_detection.py`: computes compact per-box base-frame bearing bounds in the vision owner using `effective_geometry()` and frame `sensor_crop`; publishes explicit geometry state.
- `v3/adapters/process_vision_port.py`: injects the geometry SSOT into the child person detector.
- `v3_hardware_runtime.py`: injects the same geometry on the direct/fallback camera path.
- `v3/adapters/live_person_detection.py`: sends only bounded bearing/state fields into L0 and degrades capability health for DEGRADED/INVALID geometry.
- `v3/layers/l4_world_model.py`: consumes projected bearings; `INVALID` geometry cannot become a spatial person measurement; old captures retain legacy fallback.
- `tools/v3_person_detection_test.py`: exercises the same geometry path as production and prints projection diagnostics.
- tests: adds core SSOT authority regression and feature projection/fail-closed regression.
