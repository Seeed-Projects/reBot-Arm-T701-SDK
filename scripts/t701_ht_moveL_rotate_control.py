#!/usr/bin/env python3
"""左臂、右臂或双臂：VR 初始姿态下的五秒连续末端旋转演示。"""

import os

import numpy as np
from scipy.spatial.transform import Rotation

from Lib import T701HT
from Lib.arm_pose_presets import legacy_tool_rotation, vr_initial_pose
from Lib.arm_shutdown import register_arm, return_home_on_exit
from Lib.keyboard_arm_utils import move_selected_joints, select_arm_specs
from Lib.timed_cartesian_demo import (
    DEMO_DURATION, DEMO_KP, DEMO_KD, execute_demo, plan_demo, smooth_spline,
)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def rotation_vectors(mirror_sign):
    """按旧夹爪局部轴保留 Z/Y/X 旋转及圆锥运动，随后连续返回初始姿态。"""
    local = np.eye(3)
    rotations = [local.copy()]
    for xyz in ([0., 0., mirror_sign * np.pi / 6.],
                [0., np.pi / 12., 0.], [-mirror_sign * np.pi / 12., 0., 0.]):
        local = local @ Rotation.from_euler("xyz", xyz).as_matrix()
        rotations.append(local.copy())
    rotations.append(np.eye(3))
    for phase in np.linspace(0., 2. * np.pi, 9):
        turn = Rotation.from_euler("z", mirror_sign * phase).as_matrix()
        tilt = Rotation.from_euler("y", np.pi / 18.).as_matrix()
        rotations.append(turn @ tilt @ turn.T)
    rotations.append(np.eye(3))
    return Rotation.from_matrix(np.asarray(rotations)).as_rotvec()


def build_rotation_path(robots, targets):
    bases, curves = {}, {}
    for label, robot, mirror_sign in robots:
        current = robot.forward_kinematics(targets[label])
        if current is None:
            raise RuntimeError(f"{label}无法计算 VR 初始位姿")
        bases[label] = (np.asarray(current["position"]), np.asarray(current["rotation"]))
        vectors = rotation_vectors(mirror_sign)
        # 较大的旋转分配更多时间，避免均分时间导致大角度段骤然加速。
        weights = np.sqrt(np.linalg.norm(np.diff(vectors, axis=0), axis=1))
        times = np.r_[0., np.cumsum(weights)]
        times *= DEMO_DURATION / times[-1]
        curves[label] = smooth_spline(times, vectors)

    def pose_at(label, _mirror_sign, elapsed, scale):
        position, rotation = bases[label]
        relative = Rotation.from_rotvec(curves[label](elapsed) * scale).as_matrix()
        return position, rotation @ legacy_tool_rotation(relative)

    return pose_at


@return_home_on_exit(control_mode="mit", kp=DEMO_KP, kd=DEMO_KD)
def main():
    robots = []
    try:
        for label, config, mirror_sign in select_arm_specs(SCRIPT_DIR):
            robot = register_arm(T701HT(config), label)
            robots.append((label, robot, mirror_sign))
        targets = {label: vr_initial_pose(sign < 0.) for label, _, sign in robots}
        plans = plan_demo(robots, build_rotation_path(robots, targets), targets)
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
