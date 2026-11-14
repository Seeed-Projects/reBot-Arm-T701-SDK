#!/usr/bin/env python3
"""T701-HT 关节阻抗、重力前馈与摩擦补偿。 / T701-HT joint impedance with gravity and friction compensation."""

import numpy as np

from t701_ht_joint_impedance_with_gravity import impedance_torque
from t701_ht_torque_demo import run_torque_demo, torque_demo_parser


COULOMB_FRICTION = np.array([0.20, 0.20, 0.15, 0.15, 0.0, 0.1, 0.0])
VISCOUS_FRICTION = np.array([0.06, 0.06, 0.04, 0.04, 0.0, 0.03, 0.0])
FRICTION_VELOCITY_THRESHOLD = 0.02


def gravity_friction_impedance_torque(side, robot, position, states):
    velocity = np.array([state.velocity for state in states])
    torque = impedance_torque(side, robot, position, velocity)
    return torque + robot.get_friction_compensation(
        velocity, COULOMB_FRICTION, VISCOUS_FRICTION, FRICTION_VELOCITY_THRESHOLD
    )


def main():
    args = torque_demo_parser(__doc__).parse_args()
    return run_torque_demo(
        args.arm,
        gravity_friction_impedance_torque,
        "T701-HT 关节阻抗与摩擦补偿 / Joint impedance with friction compensation",
    )


if __name__ == "__main__":
    raise SystemExit(main())
