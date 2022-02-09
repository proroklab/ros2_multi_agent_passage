from typing import Dict, List, Tuple, Type, Any
from rclpy.task import Future
from rclpy.node import Node
from evaluation_infrastructure.qos_profiles import mode_service_qos_profile
from evaluation_msgs.srv import ModeServer
from std_msgs.msg import String
import time


class Agent(Node):
    iters = 0
    MODE = ModeServer.Request()
    _global_mode: int = MODE.RESETTING
    _agent_modes: Dict[str, int] = {}
    _prev_global_mode: int = -1
    _prev_agent_modes: Dict[str, int] = {}

    _mode_future: Dict[str, Future] = {}
    _mode_future_timestamp: Dict[str, float] = {}

    def __init__(self):
        super().__init__("agent")

        self.declare_parameter("cycle_frequency")
        cycle_frequency = self.get_parameter("cycle_frequency").value

        self.update_timer = self.create_timer(1.0 / cycle_frequency, self._run)
        self.sync_mode_timer = self.create_timer(1.0, self._sync__agent_modes)
        self.mode_client = self.create_client(
            ModeServer, "/mode_server", qos_profile=mode_service_qos_profile
        )

    def _run(self):
        controllable_agents = self.get_controllable_agents()
        state_dict = self.get_state()
        self._initialize__agent_modes(controllable_agents)

        if self._global_mode == self.MODE.RESETTING:
            # Resetting agents should be NEET_RESET or RESETTING
            # Once agents have reset, they should be FINISHED_RESETTING
            for agent in controllable_agents:
                if self._agent_modes[agent] in [
                    self.MODE.FINISHED_RUNNING,
                    self.MODE.ABORT_RUNNING,
                    self.MODE.RUNNING,
                ]:
                    self._agent_modes[agent] = self.MODE.NEED_RESET

            for agent in controllable_agents:
                if self._agent_modes[agent] == self.MODE.NEED_RESET:
                    if self.queried_next_episode(agent):
                        self._agent_modes[agent] = self.MODE.RESETTING

            running_agents = [
                agent
                for agent, mode in self._agent_modes.items()
                if mode == self.MODE.RESETTING or mode == self.MODE.FINISHED_RESETTING
            ]
            if not len(running_agents) == len(controllable_agents):
                self.get_logger().info(
                    "Awaiting other agents to finish resetting...", once=True
                )
                return

            dones = self.reset(controllable_agents, state_dict)

            for agent, done in dones.items():
                # Allow jumping back to resetting mode
                if done:
                    self._agent_modes[agent] = self.MODE.FINISHED_RESETTING
                else:
                    self._agent_modes[agent] = self.MODE.RESETTING

            self.iters = 0

        elif self._global_mode == self.MODE.RUNNING:
            for agent in controllable_agents:
                if self._agent_modes[agent] == self.MODE.FINISHED_RESETTING:
                    self.get_logger().info(f"{agent} is reset")
                    self._agent_modes[agent] = self.MODE.RUNNING

            dones = self.step(controllable_agents, state_dict)

            running_agents = [
                agent
                for agent, mode in self._agent_modes.items()
                if mode == self.MODE.RUNNING or mode == self.MODE.FINISHED_RUNNING
            ]
            if not running_agents:
                self.get_logger().info(
                    "Awaiting other agents to finish the episode...", once=True
                )
                return

            for agent, done in dones.items():
                if done:
                    self._agent_modes[agent] = self.MODE.FINISHED_RUNNING

            self.iters += 1

    def get_controllable_agents(self) -> List[str]:
        """Override me. Return uuids of the agents we control"""
        return []

    def reset(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        """Override me. Execute resetting operation for the specified agents."""
        return {uuid: True for uuid in controllable_agents}

    def queried_next_episode(self, uuid: str) -> bool:
        return True

    def step(
        self, controllable_agents: List[str], state_dict: Dict[str, Dict]
    ) -> Dict[str, bool]:
        """Step function called periodically for executing policy.
        Returns dictionary of done flags for each controllable agent."""
        return {uuid: True for uuid in controllable_agents}

    def get_state(self) -> Dict[str, Dict]:
        return {}

    def _initialize__agent_modes(self, controllable_agents: List[str]) -> None:
        for agent in controllable_agents:
            if agent not in self._agent_modes:
                # Set the initial mode to abort running. In case the inference
                # node crashes, we don't have to restart the state server, which
                # will transition into global resetting.
                self._agent_modes[agent] = self.MODE.ABORT_RUNNING

    def _sync__agent_modes(self) -> None:
        controllable_agents = self.get_controllable_agents()
        self._initialize__agent_modes(controllable_agents)

        for agent in controllable_agents:
            if agent in self._mode_future and (
                self._mode_future[agent].done() or self._mode_future[agent].cancelled()
            ):
                if self._mode_future[agent].result() is not None:
                    self._global_mode = self._mode_future[agent].result()._global_mode
                self._mode_future.pop(agent, None)
                self._mode_future_timestamp.pop(agent, None)

        for agent in controllable_agents:
            if agent in self._mode_future:
                if (time.time() - self._mode_future_timestamp[agent]) > 5.0:
                    self._mode_future[agent].cancel()
                    self.get_logger().debug(f"Cancelled mode future for {agent}")
                continue
            req = ModeServer.Request()
            req.uuid = String(data=agent)
            req.current_mode = self._agent_modes[agent]
            self._mode_future[agent] = self.mode_client.call_async(req)
            self._mode_future_timestamp[agent] = time.time()

        for agent in controllable_agents:
            if agent not in self._agent_modes:
                continue

            if agent not in self._prev_agent_modes:
                self._prev_agent_modes[agent] = -1
            if self._agent_modes[agent] != self._prev_agent_modes[agent]:
                self.on_agent_mode_transition(
                    agent, self._prev_agent_modes[agent], self._agent_modes[agent]
                )
                self._prev_agent_modes[agent] = self._agent_modes[agent]

        if self._global_mode != self._prev_global_mode:
            self.on__global_mode_transition(self._prev_global_mode, self._global_mode)
            self._prev_global_mode = self._global_mode

    def on_agent_mode_transition(self, controllable_agent: str, old_mode, new_mode):
        self.get_logger().debug(
            f"agent {controllable_agent} mode transition from {old_mode} to {new_mode}"
        )

    def on__global_mode_transition(self, old_mode, new_mode):
        self.get_logger().debug(f"global mode transition from {old_mode} to {new_mode}")
