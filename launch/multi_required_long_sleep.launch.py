import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    heavy_node = ExecuteProcess(
        cmd=["sleep", "1000"],
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
            on_exit=launch.actions.Shutdown()),
        heavy_node
    ])
