"""Explicit offline fixtures for tests that previously used Python tuning defaults.

This file is never imported by production. Integration tests use ConfigResolver
and the checked-in robot config; these historical unit fixtures keep isolated
algorithm tests independent of robot tuning.
"""
from __future__ import annotations

from dataclasses import is_dataclass, replace
from functools import lru_cache
import inspect
import json
from pathlib import Path
from typing import get_type_hints

_DEFAULTS = json.loads((Path(__file__).parent / "fixtures/v3_unit_config.json").read_text())


def _complete(value):
    if isinstance(value, dict):
        return {k:_complete(v) for k,v in {**_DEFAULTS.get(value.get("__type__"), {}), **value}.items()}
    if isinstance(value, list):
        return [_complete(v) for v in value]
    return value


@lru_cache(maxsize=1)
def robot_config():
    from v3.config import ConfigResolver
    return ConfigResolver.for_project((Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))).resolve()


def navigation_from_control(control):
    from v3.config import ConfigResolver
    root=(Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))/"conf"
    return ConfigResolver.from_documents(*(json.loads((root/name).read_text()) for name in ("hardver.json", "fizika.json", "speed_map.json")), control).navigation


def configured(cls, *args, **kwargs):
    from v3.replay import _decode_production_value
    name=cls.__name__
    signature=inspect.signature(cls)
    bound=signature.bind_partial(*args, **{k:v for k,v in kwargs.items() if k in signature.parameters})
    hints=get_type_hints(cls if is_dataclass(cls) else cls.__init__)
    values={k:_decode_production_value(_complete(v), hints[k], f"unit_fixture.{name}.{k}")
            for k,v in _DEFAULTS.get(name,{}).items() if k not in bound.arguments}
    if name == "TrajectoryNavigator":
        policy=values.get("async_config",kwargs.get("async_config"))
        aliases={"rollout_release_tick_gap":"release_tick_gap","rollout_release_delay_ns":"release_delay_ns",
                 "max_plan_age_ns":"max_plan_age_ns","completion_inputs":"completion_inputs","request_timeout_ns":"request_timeout_ns"}
        changes={target:kwargs.pop(source) for source,target in aliases.items() if source in kwargs}
        if changes: values["async_config"]=replace(policy,**changes)
    values.update(kwargs)
    return cls(*args,**values)


def bounded_fixture(hardware_path, physics_path, speed_map_path, command_profile, *, control_path=None, sensor_policy=None, **overrides):
    """Close explicit modified documents for isolated hardware/runtime unit tests."""
    from v3.config import ConfigResolver
    from dataclasses import asdict
    h,p,s=[json.loads(Path(path).read_text()) for path in (hardware_path,physics_path,speed_map_path)]
    control_path=control_path or (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))/'conf/vezerles.json'
    c=json.loads(Path(control_path).read_text())
    if sensor_policy is not None:
        values=asdict(sensor_policy)
        for key in ('imu_heading_clockwise_positive','imu_yaw_rate_axis','imu_yaw_rate_clockwise_positive','imu_yaw_offset_rad'):p[key]=values.pop(key)
        c['sensor_policy']=values
        c['lidar_runtime']['matcher_max_result_age_s']=values['lidar_maximum_result_age_ns']/1e9
    for key,value in overrides.items():
        if key in ('tick_period_ns','max_preflight_age_ns'):c['runtime'][key]=value
        elif key=='gpio_chip':h[key]=value
        elif key=='pwm_frequency_hz':c['motor'][key]=value
        else:raise TypeError(key)
    return ConfigResolver.from_documents(h,p,s,c).bounded(command_profile)
