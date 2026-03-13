from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


@dataclass(frozen=True)
class ContextConfig:
    node_name: str = "cfswarm_sdk"


class RosContext:
    """
    Minimal ROS2 lifecycle manager for the controller process.

    It only provides:
      - rclpy.init()
      - Node creation
      - Executor spin in a background thread
      - clean shutdown

    Rule:
      SDK use RosContext, user code must NOT call rclpy.init()/spin() directly.
    """

    def __init__(self, cfg: ContextConfig | None = None) -> None:
        self._cfg = cfg or ContextConfig()
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def node(self) -> Node:
        if self._node is None:
            raise RuntimeError("RosContext not started")
        return self._node

    def start(self) -> Node:
        if rclpy.ok():
            raise RuntimeError(
                "rclpy already initialized in this process. "
                "Use a single RosContext, or manage rclpy yourself ."
            )
        rclpy.init()
        self._node = rclpy.create_node(self._cfg.node_name)

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)

        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        return self._node

    def shutdown(self) -> None:
        if self._executor and self._node:
            self._executor.remove_node(self._node)
        if self._node:
            self._node.destroy_node()

        self._executor = None
        self._node = None

        if rclpy.ok():
            rclpy.shutdown()