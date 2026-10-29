# Hexacopter Fault Tolerant Control

Simulation of a hexacopter hovering at 10 m that loses a motor and recovers. Two approaches are compared:

- **Control re-allocation:** cascaded PID + weighted control allocation. The fault is assumed known immediately.
- **ADRC:** active disturbance rejection control. The controller is not told about the fault and treats it, along with an unknown off-centre payload, as a disturbance.

**Simulation videos (ROS 2 + RViz):** [Google Drive folder](https://drive.google.com/drive/folders/1Yvr0sd7mPDu3fAa-QyxSlOduzrMHX4zT?usp=sharing)

**Results report:** [REPORT.md](REPORT.md)

## Structure

- `python/`: standalone simulations
  - `hexa_ftc.py` is the re-allocation study, with results in `python/results/`.
  - `hexa_adrc.py` is the ADRC study, with results in `python/results_adrc/`.
- `ros2_ws/`: ROS 2 package `hexa_ftc_ros`, with a plant node, an FTC node and RViz visualisation.
  - Results are in `ros2_ws/results_ros/` and `ros2_ws/results_adrc_ros/`.

## Python

```bash
pip install numpy scipy matplotlib
python python/hexa_ftc.py
python python/hexa_adrc.py
```

`hexa_ftc.py` runs the no-fault, disturbance and motor 1–6 failure cases. `hexa_adrc.py` runs the fault-severity sweep, the payload placement study and the noise check, which takes a few minutes.

## ROS 2

```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch hexa_ftc_ros hexa_ftc.launch.py fault_motor:=1
```

Useful launch arguments:

- `fault_motor` (0–6), `fault_eta` (thrust loss 0–1), `fault_time`, `t_end`
- `controller` (`pid` or `adrc`), `use_ftc` (`false` = fault unknown)
- `payload_mass`, `payload_r`, `payload_angle`
- `disturbance`, `rviz`

Example: fault-blind ADRC with an unknown payload.

```bash
ros2 launch hexa_ftc_ros hexa_ftc.launch.py controller:=adrc use_ftc:=false fault_motor:=1 fault_time:=5.0 t_end:=15.0 payload_mass:=0.2 payload_r:=0.1 payload_angle:=90
```

On WSL, run `ros2 daemon stop && ros2 daemon start` first if the simulation does not start (the nodes do not find each other).
