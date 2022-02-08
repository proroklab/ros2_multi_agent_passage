from typing import Dict, List
from rclpy.task import Future
from evaluation_msgs.srv import InitialPoseStartGoal
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from evaluation_infrastructure.agent import Agent
from evaluation_infrastructure.qos_profiles import initial_state_service_qos_profile


class AgentStartGoal(Agent):
    reset_futures: Dict[str, Future] = {}

    trial_id = 0
    episode_id = 0

    goal_poses: Dict[str, Pose] = {}
    start_poses: Dict[str, Pose] = {}

    def __init__(self):
        super().__init__()

        self.init_state_srv = self.create_client(
            InitialPoseStartGoal,
            "/initial_state",
            qos_profile=initial_state_service_qos_profile,
        )

    def queried_next_episode(self, uuid: str) -> bool:
        if uuid not in self.reset_futures:
            self.get_logger().info(f"Requesting initial state for {uuid}...")
            req = InitialPoseStartGoal.Request()
            req.uuid = String(data=uuid)
            self.reset_futures[uuid] = self.init_state_srv.call_async(req)
            return False
        elif self.reset_futures[uuid].done():
            res = self.reset_futures[uuid].result()
            self.start_poses[uuid] = res.start
            self.goal_poses[uuid] = res.goal
            self.trial_id = res.trial_id
            self.episode_id = res.episode_id

            self.reset_futures.pop(uuid)
            return True

    def reset(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        dones = {}
        for uuid in controllable_agents:
            print("RES", uuid)
            # Run RVO, return True when done (arrived at goal) otherwise False
            dones[uuid] = True
        return dones
