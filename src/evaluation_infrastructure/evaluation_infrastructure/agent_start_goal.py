from typing import Dict, List
from evaluation_msgs.srv import InitialPoseStartGoal
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from evaluation_infrastructure.agent import Agent
from evaluation_infrastructure.qos_profiles import initial_state_service_qos_profile
from evaluation_msgs.action import PoseControl
from rclpy.action import ActionClient
from rclpy.task import Future
from functools import partial


class AgentStartGoal(Agent):
    _initial_state_futures: Dict[str, Future] = {}

    trial_id = 0
    episode_id = 0

    goal_poses: Dict[str, Pose] = {}
    start_poses: Dict[str, Pose] = {}

    _action_clients_rvo_reset: Dict[str, ActionClient] = {}
    _reset_dones: Dict[str, bool] = {}

    def __init__(self):
        super().__init__()

        self.init_state_srv = self.create_client(
            InitialPoseStartGoal,
            "/initial_state",
            qos_profile=initial_state_service_qos_profile,
        )

        self.add_global_mode_transition_callback(
            self.global_mode_transition_clear_resets
        )

    def queried_next_episode(self, uuid: str) -> bool:
        if not self.init_state_srv.service_is_ready():
            self.get_logger().debug(f"Waiting for initial state service")
            return False

        if uuid not in self._initial_state_futures:
            self.get_logger().info(f"Requesting initial state for {uuid}...")
            req = InitialPoseStartGoal.Request()
            req.uuid = String(data=uuid)
            self._initial_state_futures[uuid] = self.init_state_srv.call_async(req)
            return False
        elif self._initial_state_futures[uuid].done():
            res = self._initial_state_futures[uuid].result()
            self.start_poses[uuid] = res.start
            self.goal_poses[uuid] = res.goal
            self.trial_id = res.trial_id
            self.episode_id = res.episode_id
            self._initial_state_futures.pop(uuid)
            return True

    def goal_response_callback(self, uuid, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Rejected navigation request")
            self._reset_dones[uuid] = True
            return

        def get_result_callback(_):
            self._reset_dones[uuid] = True

        future = goal_handle.get_result_async()
        future.add_done_callback(get_result_callback)

    def global_mode_transition_clear_resets(self, old_mode, new_mode):
        if new_mode == self.MODE.RESETTING:
            self._reset_dones = {}

    def reset(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        for agent in controllable_agents:
            if agent not in self._action_clients_rvo_reset:
                self._action_clients_rvo_reset[agent] = ActionClient(
                    self, PoseControl, f"/{agent}/pose_control"
                )

            if (
                self._action_clients_rvo_reset[agent].server_is_ready()
                and agent not in self._reset_dones
            ):
                g = PoseControl.Goal()
                g.goal_pose = self.start_poses[agent]
                future = self._action_clients_rvo_reset[agent].send_goal_async(g)
                future.add_done_callback(partial(self.goal_response_callback, agent))
                self._reset_dones[agent] = False

        return self._reset_dones
