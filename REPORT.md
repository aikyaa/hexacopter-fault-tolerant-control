# Report: Hexacopter Fault Tolerant Control by Control Re-allocation

🎥 **Simulation video (all cases, ROS 2 + RViz):** [Google Drive](https://drive.google.com/file/d/1snuhvErvYkib8klGl688AJ4HS31CERCj/view?usp=sharing)

## 1. Objective

Keep a hexacopter in stable hover after one motor fails completely, by redistributing the control effort over the remaining five motors (control re-allocation). The fault is assumed to be known as soon as it happens (no fault detection and isolation).

## 2. Setup

| Item | Value |
|---|---|
| Mass, arm length | 0.95 kg, 0.33 m |
| Inertia (Jxx, Jyy, Jzz) | 6.89e-3, 6.89e-3, 3.44e-2 kg·m² |
| Thrust / drag coefficient (kf, km) | 3.13e-4, 7.5e-6 |
| Motor limit, time constant | 110 rad/s, 0.10 s |
| Hover motor speed | 70.4 rad/s |
| Control loop | 100 Hz (Ts = 0.01 s) |
| Scenario | Hover at 10 m, total motor failure at t = 2 s, 10 s run |

**Controller:** a PX4-style cascade. Attitude uses angle P → rate PI, and altitude uses position P → climb-rate PID. The controller outputs the desired moments M = [τφ, τθ, τψ, T].

**Allocation:** motor speeds come from M = G·ω².
- Healthy: weighted pseudo-inverse, ω² = (Ws·G)⁺·Ws·M, with Ws = diag(0.3, 0.3, 0.15, 0.25).
- After a fault: the failed motor's column of G is set to zero (Gᵢ), and the same weighted pseudo-inverse is used with Gᵢ.
- If any motor would saturate, a bounded least-squares solve (0 ≤ ω² ≤ ω_max²) is used instead.

**Cases:** no fault, no fault + disturbance (a torque gust during 3–4 s and a −2 N vertical push during 6–7 s), and failure of each motor 1–6. Every fault case is run with and without re-allocation.

## 3. Controllability and control authority

The rank of the controllability matrix at hover is **4 for the healthy vehicle and for every single-motor failure**, so the vehicle remains controllable.

The available torque at hover thrust drops after a failure, though, and becomes one-sided on some axes:

![Control authority](python/results/control_authority.png)

| Case | Roll (Nm) | Pitch (Nm) | Yaw (Nm) |
|---|---|---|---|
| Healthy | ±1.87 | ±2.16 | ±0.223 |
| M1 fail | 0 … +1.87 | ±1.67 | 0 … +0.223 |
| M3 fail | −1.87 … 0 | 0 … +2.16 | 0 … +0.223 |

Every failure leaves **yaw torque in one direction only**, and roll or pitch also lose one direction (results for all six motors are in [control_authority.csv](python/results/control_authority.csv)). This explains the yaw behaviour in Section 4.

## 4. Results

### Summary (Python simulation)

| Case | Crash without re-alloc. | Crash with | Max roll (°) | Max pitch (°) | Final yaw (°) | Min altitude (m) | Settle time (s) | Final motor speeds (rad/s) |
|---|---|---|---|---|---|---|---|---|
| No fault | – | No | 0.0 | 0.0 | 0.0 | 10.00 | 0.0 | 70 ×6 |
| No fault + disturbance | – | No | 19.8 | 12.7 | −0.1 | 9.70 | 8.0* | 70 ×6 |
| Motor 1 | Yes (t = 2.22 s) | No | 27.4 | 3.1 | 15.4 | 9.93 | 1.64 | 0 0 86 86 86 86 |
| Motor 2 | Yes (t = 2.22 s) | No | 27.4 | 3.1 | −15.4 | 9.93 | 1.63 | 0 0 86 86 86 86 |
| Motor 3 | Yes (t = 2.22 s) | No | 14.7 | 23.4 | 15.2 | 9.93 | 1.28 | 86 86 0 0 86 86 |
| Motor 4 | Yes (t = 2.22 s) | No | 14.3 | 23.7 | −15.6 | 9.93 | 1.65 | 86 86 0 0 86 86 |
| Motor 5 | Yes (t = 2.22 s) | No | 14.7 | 23.4 | −15.2 | 9.93 | 1.28 | 86 86 86 86 0 0 |
| Motor 6 | Yes (t = 2.22 s) | No | 14.3 | 23.7 | 15.6 | 9.93 | 1.65 | 86 86 86 86 0 0 |

Settle time is measured from the fault until roll and pitch stay below 2° and the altitude error stays below 5 cm.
\*For the disturbance case it is measured from t = 2 s and is dominated by the push at 6–7 s.

### With vs. without re-allocation

![Roll with and without re-allocation](python/results/compare_with_without_realloc.png)

With the normal mixer, the vehicle flips (|roll| or |pitch| > 90°) only **0.22 s after any motor failure**. With re-allocation, every case recovers to level hover within about 1.3–1.7 s.

### Motor 1 failure in detail

![Motor 1 failure](python/results/motor1_fault.png)

- Roll peaks at 27° and returns to level. Altitude dips by only 6.6 cm.
- The allocator also shuts down the **opposite motor (M2)**, and the remaining four run at 86 rad/s to carry the full weight. That is 4·86² ≈ 6·70.4², which leaves about 22% headroom below the 110 rad/s limit.
- **Yaw is not held.** The heading drifts and settles about ±15° off, because only one direction of yaw torque is left (Section 3). Roll, pitch and altitude are prioritised, which is the intended trade-off.

Compared with the thesis results for the same case (values read approximately from its figures):

| Quantity | Thesis | This sim |
|---|---|---|
| Peak roll | ~28° | 27.4° |
| Peak roll rate | ~200 °/s | 160 °/s |
| Final yaw | ~36° | 15.4° |
| Min altitude | ~9.55 m | 9.93 m |

Peak roll matches closely. Yaw and altitude deviations are smaller here, most likely because the rotor-drag yaw damping and the motor model in this sim differ from the thesis.

### Symmetry

Failures of motors 1/2 mainly disturb roll, and failures of 3–6 disturb roll and pitch together. Failures of mirrored motors give the same response magnitudes with opposite yaw sign, as expected from the airframe geometry.

## 5. ROS 2 implementation

The same plant and controller are split into a `plant_node` and an `ftc_node` that talk over ROS 2 topics, with RViz visualisation (a "ghost" drone shows the same fault without FTC). The ROS runs reproduce the Python results exactly: the same peak angles, final yaw and minimum altitude in all eight cases. Results are in [ros2_ws/results_ros/](ros2_ws/results_ros/).

## 6. Conclusions

- A hexacopter stays controllable after any single total motor failure, but it loses symmetric torque authority, most critically in yaw.
- Without re-allocation the vehicle is lost within 0.22 s. With re-allocation it recovers in under 1.7 s, with peak tilt below 28° and altitude loss below 7 cm.
- The price is an uncontrolled yaw offset of about 15° and the remaining motors running at 86 rad/s instead of 70 rad/s.
- **Limitations:** the fault is assumed known instantly (no FDI delay), the motor failure is a total loss only, there are no sensor noise or actuator rate limits, and the altitude/attitude-only model has no horizontal position control.
