#!/bin/bash
# headless ROS 2 runs of the re-allocation cases, results in results_ros/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
pkill -f "lib/hexa_ftc_ros/"; pkill -f "bin/ros2 launch hexa_ftc_ros"; sleep 1
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1
for args in "fault_motor:=0" "fault_motor:=0 disturbance:=true" fault_motor:={1,2,3,4,5,6}; do
    echo "=== $args"
    timeout 120 ros2 launch hexa_ftc_ros hexa_ftc.launch.py rviz:=false out_dir:=results_ros $args 2>&1 | grep -E "saved|CRASH|Error"
done
