import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# launch argument name, default value, type
args = [("fault_motor", "1", int), ("fault_time", "2.0", float), ("fault_eta", "1.0", float),
        ("disturbance", "false", bool), ("t_end", "10.0", float),
        ("cg_cm", "0.0", float), ("cg_angle", "0.0", float),
        ("controller", "pid", str), ("use_ftc", "true", bool), ("out_dir", "results_ros", str),
        ("rviz", "true", bool), ("start_delay", "3.0", float), ("slowmo", "0.15", float),
        ("hold_time", "0.0", float)]


def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory("hexa_ftc_ros"), "rviz", "hexa.rviz")
    params = {}
    for name, default, typ in args:
        params[name] = ParameterValue(LaunchConfiguration(name), value_type=typ)
    plant = Node(package="hexa_ftc_ros", executable="plant_node", output="screen", parameters=[params])
    ftc = Node(package="hexa_ftc_ros", executable="ftc_node", output="screen",
               parameters=[{"use_ftc": params["use_ftc"], "controller": params["controller"]}])
    rviz = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_file],
                condition=IfCondition(LaunchConfiguration("rviz")))
    stop = [EmitEvent(event=Shutdown())]
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, default, typ in args]
        + [plant, ftc, rviz,
           RegisterEventHandler(OnProcessExit(target_action=rviz, on_exit=stop)),
           RegisterEventHandler(OnProcessExit(target_action=plant, on_exit=stop))])
