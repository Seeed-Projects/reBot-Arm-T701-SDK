# T701-HT and Pinocchio

[简体中文](README_Pinocchio_CN.md)

## 1. Prepare the hardware

Install the dependencies and check the serial port mapping. [T701-HT SDK setup](README.md)

### Safety precautions

- Before starting, make sure the arms are securely mounted and the joint zero positions are correct. Remove any payload. Use one arm for the first run.
- Motion demos may start moving as soon as you select an arm. Clear the entire range of motion, including the space overhead and the path back to zero. Stay with the robot throughout the demo and keep your hands and body clear of pinch points at the joints and grippers.
- Support the arms before starting and stopping gravity compensation so they do not drop when compensation torque is removed.
- In demos that automatically return to zero, the first `Ctrl+C` starts the return. Press it again during the return to interrupt the motion and send a stop command.
- Collision prediction in the keyboard demo starts after the arms reach the initial VR pose and uses only the URDF meshes. Leave clearance during startup and the return to zero, and allow extra space for the grippers and the actual stopping distance.

From the project root, switch to the examples directory.

```bash
cd scripts
```

After launching a demo, enter `1` for the left arm, `2` for the right arm, or `3` for both.

## 2. Kinematics and gravity compensation

| Example | Command | Expected behavior |
| --- | --- | --- |
| Live forward kinematics (FK) | `python t701_ht_forward_kinematics_test.py` | Reads joint angles and prints the end-effector position in meters and roll, pitch, and yaw in degrees about once a second. |
| FK-to-IK validation at the current pose | `python t701_ht_inverse_kinematics_test.py` | Reads joint angles, solves inverse kinematics for the current pose, and prints the end-effector position and maximum joint angle error. |
| Gravity compensation | `python t701_ht_gravity_compensation.py` | Applies `0.8 × G(q)` subject to the torque limits to make the arms easier to move by hand, and prints the compensation torque. |
| Gravity compensation with FK output | `python t701_ht_gravity_compensation_with_fk.py` | Applies `G(q)` subject to the torque limits and prints joint angles, compensation torque, and the end-effector pose. |
| Joint impedance with gravity feedforward | `python t701_ht_joint_impedance_with_gravity.py` | Moves to and holds a preset joint pose, applies gravity compensation, and prints the total torque. |
| Joint impedance with gravity and friction feedforward | `python t701_ht_joint_impedance_with_gravity_friction.py` | Moves to and holds the preset joint pose, applies gravity and friction compensation, and prints the total torque. |

The FK and IK checks only read feedback. On `Ctrl+C`, the two kinematics checks and both gravity compensation demos stop the motors directly. Both joint impedance demos return to zero before stopping.

## 3. Cartesian control

| Example | Command | Expected behavior |
| --- | --- | --- |
| IK target, joint motion, and return to zero | `python t701_ht_inv_posvel_control.py` | Returns the joints to zero first. Once the IK solution for the target pose passes the error check, moves to the target with `moveJ`, then returns to zero and stops. |
| QP arm raise, overhead circle, and return to zero | `python t701_ht_qp_ik.py` | Moves to the initial VR pose, raises the arm, traces a circle overhead, then returns to zero and stops. |
| MoveL straight, diagonal, and square paths | `python t701_ht_moveL_pos_control.py` | Holds the target end-effector orientation while tracing straight, diagonal, and square paths in succession. |
| MoveL axial rotation and conical motion | `python t701_ht_moveL_rotate_control.py` | Holds the target end-effector position while rotating about the local axes and tracing a conical motion. |

Both MoveL demos move to the initial VR pose, run the motion sequence for five seconds, then return to zero and stop. The five seconds exclude initial positioning and the return to zero.

## 4. Keyboard control

| Example | Command | Expected behavior |
| --- | --- | --- |
| End-effector pose | `python t701_ht_keyboard_cartesian_pos_control.py` | Each key press changes the target pose by one step. The arm moves when IK finds a solution and holds the previous joint target when no solution is found. |
| End-effector velocity | `python t701_ht_keyboard_cartesian_vel_control.py` | Hold a key to translate or rotate continuously. Release it to ramp the velocity command to zero. |
| End-effector velocity with mesh-based collision prediction | `python t701_ht_keyboard_cartesian_vel_collision_control.py` | Keys control end-effector velocity. The demo commands zero velocity and displays a message if it predicts a collision or joint limit violation. |
| Null-space posture lock | `python t701_ht_keyboard_cartesian_pd_nullspace_lock_control.py` | Tracks the end-effector pose set by the keys while keeping the joint posture as close as possible to the reference. |

Run these demos in a local terminal or over SSH. After arm selection, the selected arms move automatically to the initial VR pose. Use `W`/`S`, `A`/`D`, and `Q`/`E` to translate along X, Y, and Z respectively. Use `1`/`2`, `3`/`4`, and `5`/`6` to rotate about the corresponding axes. In the null-space demo, press `R` to lock the reference posture again. Press `Esc` or `Ctrl+C` to return to zero and stop.

The collision prediction demo uses the STL meshes in `T701-HT_URDF/meshes/` to predict collisions and joint limit violations over the next 0.20 seconds. In dual-arm mode, it also checks for collisions between the arms.
