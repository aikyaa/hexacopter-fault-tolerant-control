# Hexacopter Fault Tolerant Control

Simulation of a hexacopter hovering at 10 m that loses a motor at t = 2 s and recovers using control re-allocation (cascaded PID + weighted control allocation). The fault is assumed known immediately (no fault detection).

🎥 **Simulation video (all cases, ROS 2 + RViz):** [Google Drive](https://drive.google.com/file/d/1snuhvErvYkib8klGl688AJ4HS31CERCj/view?usp=sharing)

📄 **Results report:** [REPORT.md](REPORT.md)

## Structure

- `python/` – standalone simulation (`hexa_ftc.py`), results in `python/results/`
- `ros2_ws/` – ROS 2 package `hexa_ftc_ros` (plant node + FTC node, RViz visualisation), results in `ros2_ws/results_ros/`

## Python

```bash
pip install numpy scipy matplotlib
python python/hexa_ftc.py
```

Runs the no-fault, disturbance and motor 1–6 failure cases and writes CSVs and plots to `python/results/`.

## ROS 2

```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch hexa_ftc_ros hexa_ftc.launch.py fault_motor:=1
```

Useful launch arguments: `fault_motor` (0–6), `use_ftc`, `disturbance`, `rviz`, `t_end`.
