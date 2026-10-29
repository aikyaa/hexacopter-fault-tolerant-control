#!/bin/bash
# headless ROS 2 runs of the fault-blind ADRC cases, results in results_adrc_ros/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
pkill -f "lib/hexa_ftc_ros/"; pkill -f "bin/ros2 launch hexa_ftc_ros"; sleep 1
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1      # WSL: node discovery needs the daemon
run() {
    echo "=== $*"
    timeout 120 ros2 launch hexa_ftc_ros hexa_ftc.launch.py rviz:=false out_dir:=results_adrc_ros "$@" 2>&1 \
        | grep --line-buffered -E "WARN|ERROR|saved|CRASH|Traceback|Error" | head -8
}
PL="fault_time:=5.0 t_end:=15.0 payload_mass:=0.2 payload_r:=0.1"
run controller:=pid fault_motor:=1
run controller:=adrc use_ftc:=false fault_motor:=1
run controller:=adrc use_ftc:=false fault_motor:=1 fault_eta:=0.5
run controller:=pid use_ftc:=false fault_motor:=1 fault_eta:=0.25
run controller:=adrc use_ftc:=false fault_motor:=1 $PL payload_angle:=90
run controller:=adrc use_ftc:=false fault_motor:=1 $PL payload_angle:=0
run controller:=adrc use_ftc:=false fault_motor:=1 $PL payload_angle:=180
ls results_adrc_ros
