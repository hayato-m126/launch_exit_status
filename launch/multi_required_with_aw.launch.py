import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    autoware_launch_file = os.path.join(
        get_package_share_directory("autoware_launch"),
        "launch",
        "logging_simulator.launch.xml",
    )
    autoware_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(autoware_launch_file),
        launch_arguments={
            "map_path": LaunchConfiguration("map_path"),
            "vehicle_model": LaunchConfiguration("vehicle_model"),
            "sensor_model": LaunchConfiguration("sensor_model"),
            "planning": LaunchConfiguration("planning"),
        }.items(),
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument("number_of_cycles", default_value="40"),
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
        autoware_launch,
    ])
