# path: backend_ros2/src/mocap_bridge_ros2/mocap_bridge_ros2/config.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Literal, Optional


InputType = Literal[
    "pose_stamped",        # geometry_msgs/PoseStamped
    "transform_stamped",   # geometry_msgs/TransformStamped
    "odom",                # nav_msgs/Odometry
    "pose",                # geometry_msgs/Pose
    "twist_xyz",           # geometry_msgs/Twist interpreted as position in linear.{x,y,z}
]


@dataclass(frozen=True)
class SourceConfig:
    """
    Defines one input source to be normalized into:
      {topic_prefix}/{drone_id}/pose  (PoseStamped)

    input_type controls how the incoming message is interpreted.

    twist_xyz:
      - ONLY use if upstream publishes position in Twist.linear.{x,y,z}.
      - Orientation will be set to identity quaternion.
    """
    topic: str
    input_type: InputType
    frame_id: Optional[str] = None  # optional override per source


@dataclass(frozen=True)
class MocapNormalizerConfig:
    """
    ROS→ROS normalizer configuration.

    - output_frame_id: PoseStamped.header.frame_id
    - output_topic_prefix: base prefix for output topics
    - axis_mode: optional axis transform preset ("identity", "optitrack_to_enu", ...)
    - sources: mapping drone_id -> SourceConfig
    """
    output_frame_id: str
    output_topic_prefix: str
    axis_mode: str
    sources: Dict[str, SourceConfig]


def _require(d: dict, key: str):
    if key not in d:
        raise ValueError(f"Missing required config key: {key}")
    return d[key]


def load_config_from_dict(d: dict) -> MocapNormalizerConfig:
    # Keep YAML keys aligned with your previous style where possible.
    output_frame_id = str(d.get("frame_id", "map"))
    output_topic_prefix = str(d.get("topic_prefix", "/mocap")).rstrip("/")
    axis_mode = str(d.get("axis_mode", "identity"))

    sources_raw = _require(d, "sources")
    if not isinstance(sources_raw, dict):
        raise ValueError("'sources' must be a mapping: drone_id -> {topic, type}")

    sources: Dict[str, SourceConfig] = {}
    for drone_id, item in sources_raw.items():
        if not isinstance(item, dict):
            raise ValueError(f"sources.{drone_id} must be a dict")

        topic = str(_require(item, "topic"))
        input_type = str(_require(item, "type"))

        # Validate input_type early
        valid_types = set(InputType.__args__)  # type: ignore[attr-defined]
        if input_type not in valid_types:
            raise ValueError(
                f"Invalid sources.{drone_id}.type='{input_type}'. "
                f"Valid: {sorted(valid_types)}"
            )

        frame_override = item.get("frame_id")
        sources[str(drone_id)] = SourceConfig(
            topic=topic,
            input_type=input_type,  # type: ignore[arg-type]
            frame_id=str(frame_override) if frame_override is not None else None,
        )

    return MocapNormalizerConfig(
        output_frame_id=output_frame_id,
        output_topic_prefix=output_topic_prefix,
        axis_mode=axis_mode,
        sources=sources,
    )