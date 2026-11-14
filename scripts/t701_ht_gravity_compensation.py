#!/usr/bin/env python3
"""T701-HT 重力补偿示例。 / T701-HT gravity compensation demo."""

from t701_ht_torque_demo import run_torque_demo, torque_demo_parser


GRAVITY_SCALE = 0.8


def gravity_torque(_side, robot, position, _states):
    return GRAVITY_SCALE * robot.get_Gravity(position)


def main():
    args = torque_demo_parser(__doc__).parse_args()
    return run_torque_demo(args.arm, gravity_torque, "T701-HT 重力补偿 / Gravity compensation")


if __name__ == "__main__":
    raise SystemExit(main())
