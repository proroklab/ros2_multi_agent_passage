from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction, IncludeLaunchDescription


def generate_launch_description():
    uuids = [
        "robomaster_0",
        "robomaster_1",
        "robomaster_2",
        "robomaster_3",
        "robomaster_4",
    ]
    ld = []
    for uuid in uuids:
        ld.append(
            GroupAction(
                [
                    PushRosNamespace(uuid),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            "launch/freyja_robomaster.launch.yaml"
                        ),
                        launch_arguments=[("uuid", uuid)],
                    ),
                ]
            )
        )
    return LaunchDescription(ld)
