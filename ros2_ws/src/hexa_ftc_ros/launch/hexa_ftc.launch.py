import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory("hexa_ftc_ros"), "rviz", "hexa.rviz")
    plant = Node(
        package="hexa_ftc_ros", executable="plant_node", output="screen",
        parameters=[{
            "fault_motor": ParameterValue(LaunchConfiguration("fault_motor"), value_type=int),
            "fault_time": ParameterValue(LaunchConfiguration("fault_time"), value_type=float),
            "disturbance": ParameterValue(LaunchConfiguration("disturbance"), value_type=bool),
            "t_end": ParameterValue(LaunchConfiguration("t_end"), value_type=float),
            "out_dir": LaunchConfiguration("out_dir"),
            "wait_viewer": ParameterValue(LaunchConfiguration("rviz"), value_type=bool),
            "hold_end": ParameterValue(LaunchConfiguration("rviz"), value_type=bool),
            "start_delay": ParameterValue(LaunchConfiguration("start_delay"), value_type=float),
            "slowmo": ParameterValue(LaunchConfiguration("slowmo"), value_type=float),
            "ghost": ParameterValue(LaunchConfiguration("ghost"), value_type=bool)}])
    ftc = Node(
        package="hexa_ftc_ros", executable="ftc_node", output="screen",
        parameters=[{"use_ftc": ParameterValue(LaunchConfiguration("use_ftc"), value_type=bool)}])
    rviz = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_file],
                condition=IfCondition(LaunchConfiguration("rviz")))
    return LaunchDescription([
        DeclareLaunchArgument("fault_motor", default_value="1"),
        DeclareLaunchArgument("fault_time", default_value="2.0"),
        DeclareLaunchArgument("disturbance", default_value="false"),
        DeclareLaunchArgument("use_ftc", default_value="true"),
        DeclareLaunchArgument("t_end", default_value="10.0"),
        DeclareLaunchArgument("out_dir", default_value="results_ros"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("start_delay", default_value="3.0"),
        DeclareLaunchArgument("slowmo", default_value="0.15"),
        DeclareLaunchArgument("ghost", default_value="true"),
        plant, ftc, rviz,
        # with rviz: stop when rviz is closed, without rviz: stop when the sim is done
        RegisterEventHandler(OnProcessExit(target_action=rviz, on_exit=[EmitEvent(event=Shutdown())]),
                             condition=IfCondition(LaunchConfiguration("rviz"))),
        RegisterEventHandler(OnProcessExit(target_action=plant, on_exit=[EmitEvent(event=Shutdown())]),
                             condition=UnlessCondition(LaunchConfiguration("rviz"))),
    ])
