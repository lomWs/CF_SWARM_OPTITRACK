from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass(frozen=True)
class DroneConfig:
    drone_id: str
    uri: str
    ns: str
    mocap_topic: str  # should be /mocap/<id>/pose (from mocap_bridge_ros2)


@dataclass(frozen=True)
class CfBridgeConfig:
    drones: List[DroneConfig]
    with_orient: bool = False
    invert_y: bool = False
    start_hl: bool = True
    hl_only: bool = False
    speed: float = 0.30
    diag_period_sec: float = 0.5
    cache_dir: str = "./.cf_cache"


def _require(d: dict, key: str):
    if key not in d:
        raise ValueError(f"Missing required config key: {key}")
    return d[key]


def load_config_from_dict(d: dict) -> CfBridgeConfig:
    drones_raw = _require(d, "drones")
    if not isinstance(drones_raw, list) or not drones_raw:
        raise ValueError("'drones' must be a non-empty list")

    drones: List[DroneConfig] = []
    for item in drones_raw:
        if not isinstance(item, dict):
            raise ValueError("Each drones[] entry must be a dict")
        drones.append(
            DroneConfig(
                drone_id=str(_require(item, "id")),
                uri=str(_require(item, "uri")),
                ns=str(_require(item, "ns")).rstrip("/"),
                mocap_topic=str(_require(item, "mocap_topic")),
            )
        )

    return CfBridgeConfig(
        drones=drones,
        with_orient=bool(d.get("with_orient", False)),
        invert_y=bool(d.get("invert_y", False)),
        start_hl=bool(d.get("start_hl", True)),
        hl_only=bool(d.get("hl_only", False)),
        speed=float(d.get("speed", 0.30)),
        diag_period_sec=float(d.get("diag_period_sec", 0.5)),
        cache_dir=str(d.get("cache_dir", "./.cf_cache")),
    )