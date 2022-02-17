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

    def run_model(self, obs):
        comm_range = self.get_parameter("comm_range").value

        x = torch.cat(
            [obs["goal"] - obs["pos"], obs["pos"], obs["pos"] + obs["vel"]], dim=2
        )
        logits = self.model(obs["pos"], x, torch.tensor([comm_range]))

        actions = []
        for i in range(logits.shape[1]):
            action = self.sample_action_from_logit(logits[:, i])
            actions.append(action[0].numpy())

        return actions

    def step(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        self.update_pubs_and_subs(controllable_agents)
        self.update_current_side()

        if len(self._current_states) != len(controllable_agents):
            return {agent: False for agent in controllable_agents}

        obs = self.build_obs(controllable_agents)
        actions = self.run_model({k: torch.Tensor(v) for k, v in obs.items()})
        for uuid, action, vel in zip(controllable_agents, actions, obs["vel"][0]):
            if uuid not in self._vel_pubs:
                continue

            proc_action = self.action_constrain_vel_acc(action, vel)
            action_rot = self._current_side.apply(np.hstack([proc_action, [0]]))[
                :2
            ].astype(float)

            ref_state = ReferenceState()
            ref_state.vn = action_rot[0]
            ref_state.ve = action_rot[1]
            ref_state.yaw = np.pi / 2

            self._vel_pubs[uuid].publish(ref_state)

        dones = {}
        for i, agent in enumerate(controllable_agents):
            dist_goal = np.linalg.norm(obs["pos"][0][i] - obs["goal"][0][i])
            dones[agent] = dist_goal < self.get_parameter("goal_reached_dist").value
        return dones

    def get_controllable_agents(self) -> List[str]:
        """Override me. Return uuids of the agents we control"""
        return get_uuids_fast(self)


def main() -> None:
    rclpy.init()
    g = AgentCentralizedGNNPassage()
    rclpy.spin(g)
