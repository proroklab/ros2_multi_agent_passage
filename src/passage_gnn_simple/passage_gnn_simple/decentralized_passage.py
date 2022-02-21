import rclpy
from typing import Dict, List
from evaluation_infrastructure.agent_util import get_uuids_fast
from .passage import AgentGNNPassage
from freyja_msgs.msg import ReferenceState
import torch
import numpy as np
from passage_gnn_simple_msgs.msg import CommMessage
from rclpy.qos import QoSProfile
from functools import partial

msg_qos_profile = QoSProfile(
    history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    depth=10,
    reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
    # deadline=rclpy.qos.Duration(seconds=0, nanoseconds=0 * 1e6),
    # lifespan=rclpy.qos.Duration(seconds=0, nanoseconds=500 * 1e3),
    liveliness=rclpy.qos.QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    # liveliness_lease_duration=rclpy.qos.Duration(seconds=0, nanoseconds=0 * 1e6),
)


class AgentDecentralizedGNNPassage(AgentGNNPassage):
    msg_subscribers = {}
    msg_buffer = {}
    msg_publisher = None

    def __init__(self):
        super().__init__()
        self.declare_parameter("uuid")
        self.uuid = self.get_parameter("uuid").value

        self.msg_publisher = self.create_publisher(
            CommMessage, "comm_msg", qos_profile=msg_qos_profile
        )

    def msg_receive(self, uuid, msg):
        self.msg_buffer[uuid] = msg

    def update_msg_subscribers(self):
        for uuid in get_uuids_fast(self):
            if uuid == self.uuid or uuid in self.msg_subscribers:
                continue

            self.msg_subscribers[uuid] = self.create_subscription(
                CommMessage,
                f"/{uuid}/comm_msg",
                partial(self.msg_receive, uuid),
                qos_profile=msg_qos_profile,
            )

    def step(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        self.update_pubs_and_subs(controllable_agents)
        self.update_msg_subscribers()
        self.update_current_side()

        if len(self._current_states) != len(controllable_agents):
            return {agent: False for agent in controllable_agents}

        obs_raw = self.build_obs(controllable_agents)
        # extract agent dim
        obs = {k: torch.Tensor(v[0]) for k, v in obs_raw.items()}
        features = torch.cat(
            [obs["goal"] - obs["pos"], obs["pos"], obs["pos"] + obs["vel"]], dim=1
        )

        with torch.no_grad():
            msg_i = self.model.nns.encoder(features)

            own_msg = CommMessage()
            own_msg.data = msg_i[0].cpu().numpy().tolist()
            own_msg.stamp = self.get_clock().now().to_msg()
            self.msg_publisher.publish(own_msg)

            aggr = self.model.nns.gnn(torch.zeros(1, 32))
            for neighbor_msg in self.msg_buffer.values():
                msg_j = torch.Tensor(neighbor_msg.data).unsqueeze(0)
                aggr += self.model.nns.gnn(msg_j - msg_i)
            logits = self.model.nns.post(aggr)
            action = self.sample_action_from_logit(logits)
            proc_action = self.action_constrain_vel_acc(action, obs["vel"])[0]
            action_rot = self._current_side.apply(np.hstack([proc_action, [0]]))[
                :2
            ].astype(float)

            ref_state = ReferenceState()
            ref_state.vn = action_rot[0]
            ref_state.ve = action_rot[1]
            ref_state.yaw = np.pi / 2

            self._vel_pubs[self.uuid].publish(ref_state)

        dones = {}
        for i, agent in enumerate(controllable_agents):
            dist_goal = np.linalg.norm(obs["pos"][0][i] - obs["goal"][0][i])
            dones[agent] = dist_goal < self.get_parameter("goal_reached_dist").value
        return dones

    def get_controllable_agents(self) -> List[str]:
        return [self.uuid]


def main() -> None:
    rclpy.init()
    g = AgentDecentralizedGNNPassage()
    rclpy.spin(g)
