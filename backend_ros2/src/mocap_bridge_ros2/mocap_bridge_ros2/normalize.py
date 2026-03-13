from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

from geometry_msgs.msg import Pose, PoseStamped as PoseStampedMsg, TransformStamped, Twist
from nav_msgs.msg import Odometry

from .utils_frames import RigidBodyPose,transform_pose_axis_mode


@dataclass(frozen=True)
class _PoseParts:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_sec: Optional[float]  # if None, caller can use node clock


def _to_poseparts_from_pose(p: Pose) -> _PoseParts:
    return _PoseParts(
        x=float(p.position.x),
        y=float(p.position.y),
        z=float(p.position.z),
        qx=float(p.orientation.x),
        qy=float(p.orientation.y),
        qz=float(p.orientation.z),
        qw=float(p.orientation.w),
        stamp_sec=None,
    )


def _to_poseparts_from_pose_stamped(msg: PoseStampedMsg) -> _PoseParts:
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    pp = _to_poseparts_from_pose(msg.pose)
    return _PoseParts(**{**pp.__dict__, "stamp_sec": float(stamp)})


def _to_poseparts_from_transform_stamped(msg: TransformStamped) -> _PoseParts:
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    t = msg.transform.translation
    q = msg.transform.rotation
    return _PoseParts(
        x=float(t.x),
        y=float(t.y),
        z=float(t.z),
        qx=float(q.x),
        qy=float(q.y),
        qz=float(q.z),
        qw=float(q.w),
        stamp_sec=float(stamp),
    )


def _to_poseparts_from_odom(msg: Odometry) -> _PoseParts:
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    pp = _to_poseparts_from_pose(msg.pose.pose)
    return _PoseParts(**{**pp.__dict__, "stamp_sec": float(stamp)})


def _to_poseparts_from_twist_xyz(msg: Twist) -> _PoseParts:
    # Interprets Twist.linear as position (ONLY if upstream uses Twist like that).
    return _PoseParts(
        x=float(msg.linear.x),
        y=float(msg.linear.y),
        z=float(msg.linear.z),
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        stamp_sec=None,
    )


def poseparts_to_posestamped(
    drone_id: str,
    parts: _PoseParts,
    frame_id: str,
    axis_mode: str,
    now_sec: float,
) -> PoseStamped:

    stamp = parts.stamp_sec if parts.stamp_sec is not None else now_sec
    rb = RigidBodyPose(
        name=drone_id,
        x=parts.x,
        y=parts.y,
        z=parts.z,
        qx=parts.qx,
        qy=parts.qy,
        qz=parts.qz,
        qw=parts.qw,
        stamp_sec=stamp,
    )
    rb = transform_pose_axis_mode(rb, axis_mode)

    msg = PoseStamped()
    msg.header.frame_id = frame_id
    sec = int(stamp)
    nsec = int((stamp - sec) * 1e9)
    msg.header.stamp = Time(sec=sec, nanosec=nsec)

    msg.pose.position.x = rb.x
    msg.pose.position.y = rb.y
    msg.pose.position.z = rb.z
    msg.pose.orientation.x = rb.qx
    msg.pose.orientation.y = rb.qy
    msg.pose.orientation.z = rb.qz
    msg.pose.orientation.w = rb.qw
    return msg


#all type for future implementation, currently only pose_stamped and twist_xyz are supported by mocap_bridge_ros2
def normalize_any_to_poseparts(msg, input_type: str) -> _PoseParts:
    if input_type == "pose_stamped":
        return _to_poseparts_from_pose_stamped(msg)
    if input_type == "transform_stamped":
        return _to_poseparts_from_transform_stamped(msg)
    if input_type == "odom":
        return _to_poseparts_from_odom(msg)
    if input_type == "pose":
        return _to_poseparts_from_pose(msg)
    if input_type == "twist_xyz":
        return _to_poseparts_from_twist_xyz(msg)
    raise ValueError(f"Unsupported input_type: {input_type}")