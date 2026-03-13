from __future__ import annotations

from dataclasses import dataclass
from typing import Optional,Tuple


@dataclass(frozen=True)
class RigidBodyPose:
    """Pose in ROS ENU frame; timestamp is float seconds."""
    name: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_sec: float
    quality: Optional[float] = None

    
def _quat_normalize(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float, float]:
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if n <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / n, qy / n, qz / n, qw / n


def transform_pose_axis_mode(p: RigidBodyPose, axis_mode: str) -> RigidBodyPose:
    """
    Minimal axis transforms.

    - identity: pass-through (normalize quaternion)
    - optitrack_to_enu: preset placeholder; validate with your Motive axis settings.
    """
    if axis_mode == "identity":
        qx, qy, qz, qw = _quat_normalize(p.qx, p.qy, p.qz, p.qw)
        return RigidBodyPose(**{**p.__dict__, "qx": qx, "qy": qy, "qz": qz, "qw": qw})

    if axis_mode == "optitrack_to_enu":
        # Placeholder example mapping. You MUST validate.
        x, y, z = p.x, p.z, p.y
        qx, qy, qz, qw = _quat_normalize(p.qx, p.qz, p.qy, p.qw)
        return RigidBodyPose(
            name=p.name,
            x=x,
            y=y,
            z=z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            stamp_sec=p.stamp_sec,
            quality=p.quality,
        )

    raise ValueError(f"Unknown axis_mode: {axis_mode}")