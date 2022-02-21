import rclpy
from typing import Dict, List
from evaluation_infrastructure.agent_util import get_uuids_fast
from .passage import AgentGNNPassage
from freyja_msgs.msg import ReferenceState
import torch
import numpy as np


class AgentCentralizedGNNPassage(AgentGNNPassage):
    def __init__(self):
        super().__init__()

    def step(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        self.update_pubs_and_subs(controllable_agents)
        self.update_current_side()
        comm_range = self.get_parameter("comm_range").value

        if len(self._current_states) != len(controllable_agents):
            return {agent: False for agent in controllable_agents}

        obs = self.build_obs(controllable_agents)
        features = self.build_features_from_obs(obs)
        logits = self.model(obs["pos"], features, torch.tensor([comm_range]))

        for i, uuid in enumerate(controllable_agents):
            if uuid not in self._vel_pubs:
                continue

            ref_state = self.compute_ref_state(logits[0, i], obs["vel"][0, i])
            self._vel_pubs[uuid].publish(ref_state)

        return self.compute_dones(obs, controllable_agents)

    def get_controllable_agents(self) -> List[str]:
        """Override me. Return uuids of the agents we control"""
        return get_uuids_fast(self)


def main() -> None:
    rclpy.init()
    g = AgentCentralizedGNNPassage()
    rclpy.spin(g)
