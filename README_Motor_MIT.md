# T701-HT single-motor MIT control

[简体中文](README_Motor_MIT_CN.md)

## 1. Prepare the hardware

Install the environment and configure serial mapping. [T701-HT SDK setup](README.md). The left arm uses CAN1; the right arm uses CAN2.

In each new terminal, activate the installed environment and run the following commands from the project root:

```bash
conda activate t701-ht
```

### Safety precautions

> **The selected motor starts moving immediately after you run the demo command, without another confirmation. Secure the hardware, verify the motor zero, and clear the entire motion range before running. Stay with the hardware and keep hands and body clear of the motor, joints, and gripper pinch points.**

The original target amplitude is **±π rad (±180°)**. This demo does not apply URDF joint limits or gravity compensation. Verify that the selected motor and its connected mechanism allow this trajectory. The first `Ctrl+C` starts a return to zero, so leave clearance for that path as well. Press it again during homing to interrupt the return and attempt braking.

## 2. Run the demo

Run from the project root. Only one motor on the specified CAN channel is selected. The default example selects motor ID8 on the left arm's CAN1.

| Example | Command | Expected behavior |
| --- | --- | --- |
| Left arm CAN1, motor ID8 (default example) | `python scripts/motor_example/t701_ht_pos_vel_torque_kp_kd_control.py --can-port 1 --motor-id 8 --arm left` | Tracks a sinusoid using five-parameter MIT control and periodically prints position, velocity, torque, mode, and fault code. |
| Right arm CAN2, motor ID8 | `python scripts/motor_example/t701_ht_pos_vel_torque_kp_kd_control.py --can-port 2 --motor-id 8 --arm right` | Runs the same trajectory using the right-arm motor types and homing configuration. |

All three arguments are required: `--can-port` is the logical CAN channel (1–7), `--motor-id` is the motor ID (1–8), and `--arm` is `left` or `right`. The arm argument selects configuration only; it neither selects the CAN channel nor mirrors the trajectory.

## 3. MIT parameters and shutdown

```python
motor.pos_vel_tqe_kp_kd(pos, vel, 0.0, 2, 0.5)
robot.motor_send_cmd()
```

| Parameter | Original setting |
| --- | --- |
| Target position `pos` | `π × sin(2π × 0.1 × t)` rad: 0.1 Hz, with a 10-second period. |
| Target velocity `vel` | Time derivative of the position target, peaking at about 1.974 rad/s. |
| Feedforward torque | 0 Nm. |
| `Kp` / `Kd` | 2 / 0.5. |
| Loop delay | Sleeps for 0.001 seconds after each send; communication and printing add to the actual period. |

The original `time.time()` clock is retained, so the initial phase depends on the launch time. The source comment says “1.0 rad amplitude,” but the formula uses π rad. This port preserves the original code and parameters.

Press `Ctrl+C` to smoothly return the selected motor to zero, then send damping brake commands. Homing uses position, velocity, and maximum torque control. A second interrupt during homing, or invalid feedback, causes the shutdown logic to attempt braking. Leave clearance for the return path.

