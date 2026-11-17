#!/usr/bin/env python3
"""左臂、右臂或双臂：VR 初始姿态下的五秒连续笛卡尔位置演示。"""

import os

import numpy as np

from Lib import T701HT
from Lib.arm_pose_presets import vr_initial_pose
from Lib.arm_shutdown import register_arm, return_home_on_exit
from Lib.keyboard_arm_utils import move_selected_joints, select_arm_specs
from Lib.timed_cartesian_demo import (
    DEMO_DURATION, DEMO_KP, DEMO_KD, execute_demo, plan_demo, smooth_spline,
)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 保留 -X、镜像 Y、对角线和正方形路径，连成一条曲线，不再逐段停顿回位。
POSITION_OFFSETS = np.array([
    [0., 0., 0.],
    [-.05, 0., 0.],
    [-.05, .04, 0.],
    [-.03, -.03, .03],
    [-.035, 0., 0.],
    [-.035, -.035, 0.],
    [0., -.035, 0.],
    [0., 0., 0.],
])


def build_position_path(robots, targets):
    offsets = smooth_spline(np.linspace(0., DEMO_DURATION, len(POSITION_OFFSETS)), POSITION_OFFSETS)
    bases = {}
    for label, robot, _ in robots:
        current = robot.forward_kinematics(targets[label])
        if current is None:
            raise RuntimeError(f"{label}无法计算 VR 初始位姿")
        bases[label] = (np.asarray(current["position"]), np.asarray(current["rotation"]))

    def pose_at(label, mirror_sign, elapsed, scale):
        position, rotation = bases[label]
        offset = offsets(elapsed) * scale
        offset[1] *= mirror_sign
        return position + offset, rotation

    return pose_at


@return_home_on_exit(control_mode="mit", kp=DEMO_KP, kd=DEMO_KD)
def main():
    robots = []
    try:
        for label, config, mirror_sign in select_arm_specs(SCRIPT_DIR):
            robot = register_arm(T701HT(config), label)
            robots.append((label, robot, mirror_sign))
        targets = {label: vr_initial_pose(sign < 0.) for label, _, sign in robots}
        plans = plan_demo(robots, build_position_path(robots, targets), targets)
        states = [{"label": label, "robot": robot} for label, robot, _ in robots]
        move_selected_joints(
            states, targets, "移动到 VR 初始姿态", control_mode="mit",
            kp=DEMO_KP, kd=DEMO_KD,
        )
        execute_demo(robots, plans)
    except KeyboardInterrupt:
        print("\n演示已中断，进入回零收尾。", flush=True)
    except Exception as error:
        print(f"\n错误: {error}", flush=True)


if __name__ == "__main__":
    main()
