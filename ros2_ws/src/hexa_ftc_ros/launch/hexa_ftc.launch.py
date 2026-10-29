import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory("hexa_ftc_ros"), "rviz", "hexa.rviz")
    ctrl, use_ftc = LaunchConfiguration("controller"), LaunchConfiguration("use_ftc")
    is_adrc = ["'", ctrl, "' == 'adrc'"]
    ftc_on = ["'", use_ftc, "'.lower() == 'true'"]
    # title of the main drone, e.g. "ADRC (fault unknown)"
    label = PythonExpression(["('ADRC' if "] + is_adrc + [" else 'PID') + (' + re-allocation' if "]
                             + ftc_on + [" else ' (fault unknown)')"])
    # file name suffix; PID + re-allocation keeps the original names
    tag = PythonExpression(["'' if not ("] + is_adrc + [") and "] + ftc_on
                           + [" else '_' + '", ctrl, "' + ('' if "] + ftc_on + [" else '_blind')"])
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
            "ghost": ParameterValue(LaunchConfiguration("ghost"), value_type=bool),
            "fault_eta": ParameterValue(LaunchConfiguration("fault_eta"), value_type=float),
            "payload_mass": ParameterValue(LaunchConfiguration("payload_mass"), value_type=float),
            "payload_r": ParameterValue(LaunchConfiguration("payload_r"), value_type=float),
            "payload_angle": ParameterValue(LaunchConfiguration("payload_angle"), value_type=float),
            "label": ParameterValue(label, value_type=str),
            "tag": ParameterValue(tag, value_type=str)}])
    ftc = Node(
        package="hexa_ftc_ros", executable="ftc_node", output="screen",
        parameters=[{"use_ftc": ParameterValue(use_ftc, value_type=bool),
                     "controller": ctrl}])
    rviz = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_file],
                condition=IfCondition(LaunchConfiguration("rviz")))
    return LaunchDescription([
        DeclareLaunchArgument("fault_motor", default_value="1"),
        DeclareLaunchArgument("fault_time", default_value="2.0"),
        DeclareLaunchArgument("disturbance", default_value="false"),
        DeclareLaunchArgument("use_ftc", default_value="true",
                              description="true: fault known, re-allocation; false: fault unknown"),
        DeclareLaunchArgument("controller", default_value="pid", description="pid or adrc"),
        DeclareLaunchArgument("fault_eta", default_value="1.0",
                              description="thrust loss of the failed motor, 0..1 (1 = total)"),
        DeclareLaunchArgument("payload_mass", default_value="0.0", description="kg, 0 = no payload"),
        DeclareLaunchArgument("payload_r", default_value="0.0", description="payload offset from centre (m)"),
        DeclareLaunchArgument("payload_angle", default_value="0.0",
                              description="payload direction (deg): 0 toward M2, 180 toward M1"),
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
