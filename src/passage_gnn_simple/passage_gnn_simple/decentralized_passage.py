import rclpy
from typing import Dict, List
from evaluation_infrastructure.agent_util import get_uuids_fast
from .passage import AgentGNNPassage
import torch
from passage_gnn_simple_msgs.msg import CommMessage
from rclpy.qos import QoSProfile
from functools import partial
from datetime import datetime

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
    def __init__(self):
        super().__init__()
        self.declare_parameter("uuid")
        self.uuid = self.get_parameter("uuid").value

    @staticmethod
    def ros_timestamp_to_datetime(timestamp):
        return datetime.fromtimestamp(timestamp.sec + timestamp.nanosec / 1e9)

    @staticmethod
    def ros_point_to_tensor(point):
        return torch.tensor([point.x, point.y, point.z])

    def receive_messages(self) -> List[CommMessage]:
        raise NotImplementedError()

    def transmit_message(self, msg: CommMessage) -> None:
        raise NotImplementedError()

    def step(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        self.update_pubs_and_subs(controllable_agents)
        self.update_current_side()

        if len(self._current_states) != len(controllable_agents):
            return {agent: False for agent in controllable_agents}

        obs = self.build_obs(controllable_agents)
        features = self.build_features_from_obs(obs)[0, 0]

        with torch.no_grad():
            msg_i = self.model.nns.encoder(features)

            own_msg = CommMessage()
            own_msg.data = msg_i.cpu().tolist()
            own_msg.stamp = self.get_clock().now().to_msg()
            own_msg.pos.x = float(obs["pos"][0, 0, 0])
            own_msg.pos.y = float(obs["pos"][0, 0, 1])
            self.transmit_message(own_msg)

            own_time = self.ros_timestamp_to_datetime(own_msg.stamp)
            own_pos = self.ros_point_to_tensor(own_msg.pos)

            aggr = self.model.nns.gnn(torch.zeros(32))
            for neighbor_msg in self.receive_messages():
                msg_pos = self.ros_point_to_tensor(neighbor_msg.pos)
                msg_dist = torch.linalg.norm(own_pos - msg_pos)
                if msg_dist > self.get_parameter("comm_range").value:
                    continue

                msg_time = self.ros_timestamp_to_datetime(neighbor_msg.stamp)
                msg_age = (own_time - msg_time).microseconds
                if msg_age > ((1 / self.get_parameter("cycle_frequency").value) * 1e9):
                    self.get_logger().warn(f"Message age exceeds cycle frequency")

                msg_j = torch.Tensor(neighbor_msg.data)
                aggr += self.model.nns.gnn(msg_j - msg_i)
            logit = self.model.nns.post(aggr)

        ref_state = self.compute_ref_state(logit, obs["vel"][0, 0])
        self._vel_pubs[self.uuid].publish(ref_state)

        return self.compute_dones(obs, controllable_agents)

    def get_controllable_agents(self) -> List[str]:
        return [self.uuid]


class AgentDecentralizedGNNPassageROS(AgentDecentralizedGNNPassage):

    msg_subscribers = {}
    msg_buffer = {}
    msg_publisher = None

    def __init__(self):
        super().__init__()

        self.msg_publisher = self.create_publisher(
            CommMessage, "comm_msg", qos_profile=msg_qos_profile
        )

    def receive_messages(self) -> List[CommMessage]:
        msgs = list(self.msg_buffer.values())
        self.msg_buffer = {}
        return msgs

    def transmit_message(self, msg: CommMessage) -> None:
        self.msg_publisher.publish(msg)

    def ros_msg_receive(self, uuid, msg):
        self.msg_buffer[uuid] = msg

    def update_msg_subscribers(self):
        for uuid in get_uuids_fast(self):
            if uuid == self.uuid or uuid in self.msg_subscribers:
                continue

            self.msg_subscribers[uuid] = self.create_subscription(
                CommMessage,
                f"/{uuid}/comm_msg",
                partial(self.ros_msg_receive, uuid),
                qos_profile=msg_qos_profile,
            )

    def step(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        self.update_msg_subscribers()
        return super().step(controllable_agents, state_dict)


def main() -> None:
    rclpy.init()
    g = AgentDecentralizedGNNPassageROS()
    rclpy.spin(g)
