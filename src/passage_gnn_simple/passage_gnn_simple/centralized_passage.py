import rclpy
from typing import Dict, List
from evaluation_infrastructure.agent_util import get_uuids_fast
from evaluation_infrastructure.agent_start_goal import AgentStartGoal
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from freyja_msgs.msg import ReferenceState, CurrentState
from functools import partial
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default


class AgentCentralizedGNNPassage(AgentStartGoal):

    _vel_pubs: Dict[str, Publisher] = {}
    _state_subs: Dict[str, Subscription] = {}
    _current_states: Dict[str, CurrentState] = {}

    def __init__(self):
        super().__init__()

    def _update_current_state(self, uuid, msg):
        self._current_states[uuid] = msg

    def update_pubs_and_subs(self, controllable_agents: List[str]):
        for uuid in controllable_agents:
            if uuid not in self._vel_pubs:
                self._vel_pubs[uuid] = self.create_publisher(
                    ReferenceState,
                    f"/{uuid}/reference_state",
                    qos_profile_sensor_data,
                )
            if uuid not in self._state_subs:
                self._state_subs[uuid] = self.create_subscription(
                    CurrentState,
                    f"/{uuid}/current_state",
                    partial(self._update_current_state, uuid),
                    qos_profile=qos_profile_sensor_data,
                )

    def step(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        self.update_pubs_and_subs(controllable_agents)
        for uuid in controllable_agents:
            ref_state = ReferenceState()
            ref_state.vn = 1.0
            self._vel_pubs[uuid].publish(ref_state)

        return {agent: False for agent in controllable_agents}

    def get_controllable_agents(self) -> List[str]:
        """Override me. Return uuids of the agents we control"""
        return get_uuids_fast(self)


def main() -> None:
    rclpy.init()
    g = AgentCentralizedGNNPassage()
    rclpy.spin(g)
