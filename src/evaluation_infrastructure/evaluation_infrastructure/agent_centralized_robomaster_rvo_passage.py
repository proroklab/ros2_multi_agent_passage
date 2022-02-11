import rclpy
from typing import Dict, List
from evaluation_infrastructure.agent_util import get_uuids_fast
from evaluation_infrastructure.agent_start_goal import AgentStartGoal
from rclpy.action import ActionClient
from evaluation_msgs.action import PoseControl
from functools import partial


class AgentCentralizedRobomasterRVOPassage(AgentStartGoal):
    _action_clients_rvo_step: Dict[str, ActionClient] = {}
    _step_dones: Dict[str, bool] = {}

    def __init__(self):
        super().__init__()
        self.add_global_mode_transition_callback(
            self.global_mode_transition_clear_dones
        )

    def rvo_step_goal_response_callback(self, uuid, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Rejected navigation request")
            self._step_dones[uuid] = True
            return

        def get_result_callback(_):
            self._step_dones[uuid] = True

        future = goal_handle.get_result_async()
        future.add_done_callback(get_result_callback)

    def global_mode_transition_clear_dones(self, old_mode, new_mode):
        if new_mode == self.MODE.RESETTING:
            self._step_dones = {}

    def step(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        for agent in controllable_agents:
            if agent not in self._action_clients_rvo_step:
                self._action_clients_rvo_step[agent] = ActionClient(
                    self, PoseControl, f"/{agent}/pose_control"
                )

            if agent not in self._step_dones:
                g = PoseControl.Goal()
                g.goal_pose = self.goal_poses[agent]
                future = self._action_clients_rvo_step[agent].send_goal_async(g)
                future.add_done_callback(
                    partial(self.rvo_step_goal_response_callback, agent)
                )
                self._step_dones[agent] = False

        return self._step_dones

    def get_controllable_agents(self) -> List[str]:
        """Override me. Return uuids of the agents we control"""
        return get_uuids_fast(self)


def main() -> None:
    rclpy.init()
    g = AgentCentralizedRobomasterRVOPassage()
    rclpy.spin(g)
