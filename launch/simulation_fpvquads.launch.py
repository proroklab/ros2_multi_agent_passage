from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, Shutdown, LogInfo
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    sim_params = {"uuids": []}
    for i in range(2):
        sim_params["uuids"].append(f"fpvquad_{i}")
        sim_params[f"fpvquad_{i}_initial_position"] = [-0.69, 2.1 + i, 0.0]

    # Crash on node failure
    ld = LaunchDescription(
        [
            Node(
                package="simple_simulator",
                namespace="/sim",
                executable="fpvquad",
                name="simulation",
                parameters=[
                    {
                        "uuids": [f"fpvquad_{i}" for i in range(1)],
                        "fpvquad_0_initial_position": [-0.69, 2.1, -0.1],
                    }
                ],
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
