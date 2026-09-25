"""Compatibility names; ConfigResolver owns all robot config loading."""
from pathlib import Path
from v3.config_hardware import NativeSensorPolicyConfig, POSE_FRAME_ID
from v3.adapters.bounded_command import BoundedTeleopProfile, BoundedExploreProfile
from v3.composition.runtime_config import BoundedPhysicalRuntimeConfig
from v3.composition.native_control import V3NavigationConfig, V3_NAVIGATION_CONTRACT

def load_bounded_physical_runtime_config(
    hardware_path: str | Path,
    physics_path: str | Path,
    speed_map_path: str | Path,
    command_profile: BoundedTeleopProfile | BoundedExploreProfile,
    *,
    control_path: str | Path,
) -> BoundedPhysicalRuntimeConfig:
    """Compatibility entrypoint: one resolver, no separate bounded authority."""
    from v3.config import ConfigResolver
    if not isinstance(command_profile, (BoundedTeleopProfile, BoundedExploreProfile)):
        raise TypeError("command_profile must be bounded command profile")
    return ConfigResolver(hardware_path, physics_path, speed_map_path, control_path).resolve().bounded(command_profile)


def load_v3_navigation_config(path_value: str | Path) -> V3NavigationConfig:
    from v3.config import ConfigResolver
    path = Path(path_value)
    resolved = ConfigResolver(path.with_name("hardver.json"), path.with_name("fizika.json"), path.with_name("speed_map.json"), path).resolve()
    return resolved.navigation
