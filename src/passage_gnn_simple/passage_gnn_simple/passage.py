from typing import Dict, List
from evaluation_infrastructure.agent_start_goal import AgentStartGoal
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from freyja_msgs.msg import ReferenceState, CurrentState
from functools import partial
import torch
import torch_geometric
from torch_geometric.nn.conv import MessagePassing
from scipy.spatial.transform import Rotation as R
import numpy as np


class AgentGNNPassage(AgentStartGoal):

    _vel_pubs: Dict[str, Publisher] = {}
    _state_subs: Dict[str, Subscription] = {}
    _current_states: Dict[str, CurrentState] = {}
    model = None
    _current_side: R = R.from_euler("z", 0.0)

    def __init__(self):
        super().__init__()

        self.declare_parameter("model_path")
        self.model = torch.jit.load(self.get_parameter("model_path").value)
        self.declare_parameter("comm_range", 2.0)
        self.declare_parameter("max_v", 1.5)
        self.declare_parameter("max_a", 1.0)
        self.declare_parameter("goal_reached_dist", 0.25)

    def _update_current_state(self, uuid, msg):
        self._current_states[uuid] = msg

    def update_pubs_and_subs(self, controllable_agents: List[str]):
        for uuid in controllable_agents:
            if uuid not in self._vel_pubs:
                self._vel_pubs[uuid] = self.create_publisher(
                    ReferenceState,
                    f"/{uuid}/reference_state",
                    1,
                )
            if uuid not in self._state_subs:
                self._state_subs[uuid] = self.create_subscription(
                    CurrentState,
                    f"/{uuid}/current_state",
                    partial(self._update_current_state, uuid),
                    1,
                )

    def compute_dones(self, obs, controllable_agents):
        dones = {}
        for i, agent in enumerate(controllable_agents):
            dist_goal = np.linalg.norm(obs["pos"][0][i] - obs["goal"][0][i])
            dones[agent] = dist_goal < self.get_parameter("goal_reached_dist").value
        return dones

    def sample_action_from_logit(self, logit):
        assert logit.ndim == 1
        max_v = self.get_parameter("max_v").value

        mean, log_std = torch.chunk(logit, 2)
        action_dist = torch.distributions.normal.Normal(mean, torch.exp(log_std))
        return action_dist.sample().clamp(-max_v, max_v)

    def action_constrain_vel_acc(self, action, velocity):
        assert action.shape == velocity.shape
        max_v = self.get_parameter("max_v").value
        max_a = self.get_parameter("max_a").value

        f = 4.0
        clipped_v = action.clamp(-max_v, max_v)
        desired_a = (clipped_v - velocity) / (1 / f)
        possible_a = desired_a.clamp(-max_a, max_a)
        possible_v = velocity + possible_a * (1 / f)
        return possible_v.clamp(-max_v, max_v)

    def compute_ref_state(self, logit, vel):
        assert logit.ndim == 1 and vel.ndim == 1

        action = self.sample_action_from_logit(logit)
        proc_action = self.action_constrain_vel_acc(action, vel)
        action_rot = self._current_side.apply(
            torch.hstack((proc_action, torch.zeros(1)))
        )

        ref_state = ReferenceState()
        ref_state.vn = float(action_rot[0])
        ref_state.ve = float(action_rot[1])
        ref_state.yaw = np.pi / 2
        return ref_state

    def update_current_side(self):
        if list(self.start_poses.values())[0].position.y > 0.0:
            self._current_side = R.from_euler("z", np.pi)
        else:
            self._current_side = R.from_euler("z", 0.0)

    def build_obs(self, controllable_agents):
        obs = {"pos": [[]], "vel": [[]], "goal": [[]]}
        for uuid in controllable_agents:
            if uuid not in self._current_states:
                continue

            obs["goal"][0].append(
                self._current_side.apply(
                    [
                        self.goal_poses[uuid].position.x,
                        self.goal_poses[uuid].position.y,
                        0.0,
                    ]
                )[:2]
            )
            obs["pos"][0].append(
                self._current_side.apply(
                    [
                        self._current_states[uuid].state_vector[0],
                        self._current_states[uuid].state_vector[1],
                        0.0,
                    ]
                )[:2]
            )
            obs["vel"][0].append(
                self._current_side.apply(
                    [
                        self._current_states[uuid].state_vector[3],
                        self._current_states[uuid].state_vector[4],
                        0.0,
                    ]
                )[:2]
            )

        return {k: torch.Tensor(v) for k, v in obs.items()}

    def build_features_from_obs(self, obs):
        return torch.cat(
            [obs["goal"] - obs["pos"], obs["pos"], obs["pos"] + obs["vel"]], dim=2
        )
