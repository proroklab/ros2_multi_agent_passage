import rclpy
from typing import Dict, List
from evaluation_infrastructure.agent_util import get_uuids_fast
from evaluation_infrastructure.agent_start_goal import AgentStartGoal


class AgentCentralizedRobomasterRVOPassage(AgentStartGoal):
    def __init__(self):
        super().__init__()

    def get_controllable_agents(self) -> List[str]:
        """Override me. Return uuids of the agents we control"""
        return get_uuids_fast(self)


def main() -> None:
    rclpy.init()
    g = AgentCentralizedRobomasterRVOPassage()
    rclpy.spin(g)
