from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, Shutdown, LogInfo
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Crash on node failure
    sim_params = {"uuids": []}
    for i, px in enumerate([-0.5, 0.05, 1.02]):
        sim_params["uuids"].append(f"robomaster_{i}")
        sim_params[f"robomaster_{i}_initial_position"] = [-2.0, px, 0.0]

    ld = LaunchDescription(
        [
            Node(
                package="simple_simulator",
                namespace="/sim",
                executable="robomaster",
                name="simulation",
                parameters=[sim_params],
            ),
            Node(
                package="rviz2",
                namespace="/rviz",
                executable="rviz2",
                name="rviz",
                arguments=["-d", "src/simple_simulator/rviz/sim.rviz"],
            ),
        ]
    )
    shutdown = Shutdown(reason="Node failure, stopping launch...")

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                on_exit=[LogInfo(msg=["A Node crashed, shutting down..."]), shutdown]
            )
        )
    )
    return ld
