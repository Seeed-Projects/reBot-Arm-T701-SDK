#!/usr/bin/env python3
"""T701-HT 关节阻抗与重力前馈。 / T701-HT joint impedance with gravity feedforward."""

import numpy as np

from t701_ht_torque_demo import run_torque_demo, torque_demo_parser


STIFFNESS = np.array([15.0, 15.0, 10.0, 10.0, 5.0, 3.0, 5.0])
DAMPING = np.array([0.5, 0.8, 0.8, 0.2, 0.2, 0.1, 0.1])
LEFT_TARGET = np.array([-0.2, -0.2, 0.2, -0.4, 0.5, 0.2, -0.1])
TARGETS = {"left": LEFT_TARGET, "right": -LEFT_TARGET}


def impedance_torque(side, robot, position, velocity):
    if not np.all(np.isfinite(velocity)):
        raise RuntimeError(f"{side} — 关节速度反馈无效 / Invalid joint velocity feedback")

    target = TARGETS[side]
    if np.any(target < robot.joint_limits["lower"]) or np.any(target > robot.joint_limits["upper"]):
        raise ValueError(f"{side} — 阻抗目标超出关节限位 / Impedance target exceeds joint limits")

    return STIFFNESS * (target - position) - DAMPING * velocity + robot.get_Gravity(position)


def gravity_impedance_torque(side, robot, position, states):
    velocity = np.array([state.velocity for state in states])
    return impedance_torque(side, robot, position, velocity)


def main():
    args = torque_demo_parser(__doc__).parse_args()
    return run_torque_demo(args.arm, gravity_impedance_torque, "T701-HT 关节阻抗 / Joint impedance")


if __name__ == "__main__":
    raise SystemExit(main())
