import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sleep_launch_file = os.path.join(
        get_package_share_directory("launch_exit_status"),
        "launch",
        "sleep.launch.py",
    )
    sleep_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(sleep_launch_file)
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument("number_of_cycles", default_value="10"),
        launch_ros.actions.Node(
            package="demo_nodes_py",
            executable="talker_qos",
            name="talker",
            output="screen",
            arguments=["--number_of_cycles", LaunchConfiguration("number_of_cycles")],
            on_exit=launch.actions.Shutdown()),

        launch_ros.actions.Node(
            package="demo_nodes_py",
            executable="listener_qos",
            name="listener",
            output="screen",
            arguments=["--number_of_cycles", "1000"],
            on_exit=launch.actions.Shutdown()
        ),
        sleep_launch,
    ])
