from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_sec: float


@dataclass(frozen=True)
class Observation:
    poses: Dict[str, Pose3D]
    stamp_sec: float


@dataclass(frozen=True)
class VelocityCmd:
    vx: float
    vy: float
    vz: float
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class Command:
    """
    Optional command container (handy for user code).
    SDK does not execute loops; user can still use this type in their own code.
    """
    takeoff: bool = False
    land: bool = False
    ekf_reset: bool = False
    pos_abs: Optional[tuple[float, float, float]] = None
    pos_rel: Optional[tuple[float, float, float]] = None
    vel: Optional[VelocityCmd] = None