import yaml
from typing import List, Dict, Union
from evaluation_msgs.srv import InitialPoseStartGoal, ModeServer
import rclpy
from rclpy.node import Node
from evaluation_infrastructure.agent_util import get_uuids
from evaluation_infrastructure.qos_profiles import mode_service_qos_profile
from geometry_msgs.msg import Pose


class InitialStateService(Node):
    """A global initial state server for the decentralized case"""

    curr_episode_id = 0
    curr_trial_id = 0
    agent_states: List[int] = []
    global_state = ModeServer.Request().RESETTING
    agent_relations: Dict[Union[str, int], Union[str, int]] = {}

    def __init__(self):
        super().__init__("initial_state_server")
        self.load()
        self.state_srv = self.create_service(
            InitialPoseStartGoal,
            "initial_state",
            self.get_initial_state,
        )
        self.episode_srv = self.create_service(
            ModeServer,
            "mode_server",
            self.state_transition,
            qos_profile=mode_service_qos_profile,
        )

    def get_initial_state(self, req, resp):
        ep = self.curr_ep()
        idx = self.agent_relations[req.uuid.data]

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

        resp.start = dict_to_pose(ep[idx]["start"])
        resp.goal = dict_to_pose(ep[idx]["goal"])
        self.get_logger().info(f"pos {resp.start}")
        resp.episode_id = self.curr_episode_id
        resp.trial_id = self.curr_trial_id
        return resp

    def state_transition(self, req, resp):
        """Handle state transitions for all agents and
        synchronize them.
        TODO: This thing is similar in the other state server. See if this can be extracted.
        """
        idx = self.agent_relations[req.uuid.data]
        if self.agent_states[idx] != req.current_mode:
            self.agent_states[idx] = req.current_mode
            self.get_logger().info(f"states {self.agent_states}, {self.global_state}")

        # If all agents are done running, it is time to reset
        if (
            all([s == req.FINISHED_RUNNING for s in self.agent_states])
            or any([s == req.ABORT_RUNNING for s in self.agent_states])
        ) and self.global_state != req.RESETTING:
            self.global_state = req.RESETTING
            # Advance episode
            if self.curr_episode_id == self.run_n_episodes - 1:
                if self.curr_trial_id == self.run_n_trials - 1:
                    self.global_state = req.EPISODES_FINISHED
                    self.get_logger().info(
                        f"All agents finished final episode {self.curr_episode_id}, trial {self.curr_trial_id}"
                    )
                else:
                    self.get_logger().info(
                        f"Finished all episodes in trial {self.curr_trial_id} of {self.run_n_trials}"
                    )
                    self.curr_episode_id = 0
                    self.curr_trial_id += 1

            else:
                self.curr_episode_id += 1
                self.get_logger().info(
                    f"Global mode is now RESETTING to episode {self.curr_episode_id}, trial {self.curr_trial_id}"
                )
        # If all agents are done resetting, switch to run mode
        elif (
            all([s == req.FINISHED_RESETTING for s in self.agent_states])
            and self.global_state != req.RUNNING
        ):
            self.global_state = req.RUNNING
            self.get_logger().info("Global mode is now RUNNING")

        resp.global_mode = self.global_state
        return resp

    def curr_ep(self):
        return self.episodes[self.curr_episode_id]

    def load(self) -> None:
        self.declare_parameter("episodes_path")
        path = self.get_parameter("episodes_path").value

        with open(path, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)

        self.declare_parameter("n_agents")
        self.num_agents = self.get_parameter("n_agents").value
        self.declare_parameter("n_episodes")
        self.run_n_episodes = self.get_parameter("n_episodes").value
        self.declare_parameter("n_trials")
        self.run_n_trials = self.get_parameter("n_trials").value

        try:
            self.episodes = data["episodes"]
            self.agent_dones = [False] * self.num_agents
            self.agent_states = [ModeServer.Request().RESETTING] * self.num_agents
        except Exception:
            self.get_logger().error("Failed to parse episodes yaml")
            raise

        self.uuids = sorted(get_uuids(self))
        while len(self.uuids) < self.num_agents:
            self.get_logger().info(
                f"Awaiting {self.num_agents} agents online, currently {len(self.uuids)} ({self.uuids})"
            )
            self.uuids = sorted(get_uuids(self))

        # Validate
        for episode in self.episodes:
            assert (
                len(episode) >= self.num_agents
            ), "Check your config, there should be at least n_agents start_positions"

        assert self.run_n_episodes <= len(
            self.episodes
        ), f"run_n_episodes must be smaller than the number of episodes"

        assert self.num_agents == len(
            self.uuids
        ), f"Parameters require {self.num_agents} agents, but we detected \
            ({len(self.uuids)}) {self.uuids} on the network"

        # Build 2way map uuid<->agent_index
        for i in range(len(self.uuids)):
            self.agent_relations[i] = self.uuids[i]
            self.agent_relations[self.uuids[i]] = i


def main(args=None):
    rclpy.init(args=args)
    s = InitialStateService()
    rclpy.spin(s)


if __name__ == "__main__":
    main()
