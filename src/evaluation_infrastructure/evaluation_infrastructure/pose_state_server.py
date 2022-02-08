import yaml
from typing import List, Dict
from evaluation_msgs.srv import InitialPoseStartGoal
import rclpy
from evaluation_infrastructure.mode_server import ModeService
from geometry_msgs.msg import Pose
from evaluation_infrastructure.qos_profiles import initial_state_service_qos_profile


class InitialStateService(ModeService):
    episodes: List[List[Dict]] = []

    def __init__(self):
        super().__init__()

        self.declare_parameter("episodes_path")
        with open(self.get_parameter("episodes_path").value, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        self.episodes = data["episodes"]

        # Validate
        for episode in self.episodes:
            assert (
                len(episode) >= self.num_agents
            ), "Check your config, there should be at least n_agents start_positions"

        assert self.run_n_episodes <= len(
            self.episodes
        ), f"run_n_episodes must be smaller than the number of episodes"

        self.state_srv = self.create_service(
            InitialPoseStartGoal,
            "initial_state",
            self.get_initial_state,
            qos_profile=initial_state_service_qos_profile,
        )

    @staticmethod
    def dict_to_pose(d):
        p = Pose()
        p.position.x = d["position"]["x"]
        p.position.y = d["position"]["y"]
        p.position.z = d["position"]["z"]
        p.orientation.x = d["orientation"]["x"]
        p.orientation.y = d["orientation"]["y"]
        p.orientation.z = d["orientation"]["z"]
        p.orientation.w = d["orientation"]["w"]
        return p

    def get_initial_state(self, req, resp):
        ep = self.episodes[self.curr_episode_id]
        idx = self.agent_relations[req.uuid.data]
        resp.start = self.dict_to_pose(ep[idx]["start"])
        resp.goal = self.dict_to_pose(ep[idx]["goal"])
        self.get_logger().info(f"pos {resp.start}")
        resp.episode_id = self.curr_episode_id
        resp.trial_id = self.curr_trial_id
        return resp


def main(args=None):
    rclpy.init(args=args)
    s = InitialStateService()
    rclpy.spin(s)


if __name__ == "__main__":
    main()
