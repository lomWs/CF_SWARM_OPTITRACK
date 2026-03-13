from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from rclpy.node import Node

from .optitrack import OptiTrack, OptiTrackConfig
from .crazyflie_agent import CrazyflieAgent, AgentConfig


@dataclass(frozen=True)
class SwarmConfig:
    mocap_pose_topics: Dict[str, str]         # drone_id -> /mocap/<id>/pose
    agents: Dict[str, AgentConfig]            # drone_id -> agent cfg


class Swarm:
    def __init__(self, node: Node, cfg: SwarmConfig) -> None:
        self.__agents: Dict[str, CrazyflieAgent] = {did: CrazyflieAgent(node, acfg) for did, acfg in cfg.agents.items()}

    @property
    def agents(self) -> Dict[str, CrazyflieAgent]:

        return self.__agents 
    def agent(self, drone_id: str) -> CrazyflieAgent:
        try:
            return self.agents[drone_id]
        except KeyError as e:
            raise ValueError(f"Unknown agent with ID '{drone_id}'. Available agents: {list(self.agents.keys())}") from e

    def get_agent(self, drone_id: str) -> CrazyflieAgent:
        return self.agent(drone_id)
    
    def __getitem__(self, drone_id: str) -> CrazyflieAgent:
        return self.agent(drone_id) 
    