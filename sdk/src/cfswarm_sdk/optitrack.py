from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Dict, Optional

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from .types import Pose3D, Observation


def make_sensor_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


@dataclass(frozen=True)
class OptiTrackConfig:
    """
    drone_id -> topic, usually:
      /mocap/<drone_id>/pose
    """
    pose_topics: Dict[str, str]


class OptiTrack:
    """Reads canonical mocap topics from the backend."""

    def __init__(self, node: Node, cfg: OptiTrackConfig) -> None:
        self._node = node
        self._cfg = cfg

        self._lock = threading.Lock()
        self._poses: Dict[str, Pose3D] = {}
        self._stamp: float = 0.0

        qos = make_sensor_qos()
        for drone_id, topic in cfg.pose_topics.items():
            node.create_subscription(
                PoseStamped,
                topic,
                lambda msg, did=drone_id: self._on_pose(did, msg),
                qos,
            )

    def _on_pose(self, drone_id: str, msg: PoseStamped) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        pose = Pose3D(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            qx=float(q.x),
            qy=float(q.y),
            qz=float(q.z),
            qw=float(q.w),
            stamp_sec=float(stamp),
        )
        with self._lock:
            self._poses[drone_id] = pose
            if pose.stamp_sec > self._stamp:
                self._stamp = pose.stamp_sec

    def get_pose(self, drone_id: str) -> Optional[Pose3D]:
        with self._lock:
            return self._poses.get(drone_id)

    def snapshot(self) -> Observation:
        '''Consistency copy for  the latest pose for all drones, and the latest timestamp across all poses.'''
        with self._lock:
            poses = dict(self._poses)
            stamp = float(self._stamp)
        return Observation(poses=poses, stamp_sec=stamp)