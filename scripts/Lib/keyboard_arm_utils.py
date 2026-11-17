#!/usr/bin/env python3
"""键盘控制脚本共用的左右臂选择、镜像和关节准备工具。"""

import os
import time

import numpy as np

from .arm_pose_presets import VR_LEFT_JOINTS
from .arm_shutdown import register_arm


# 双臂 VR 镜像初始姿态。右臂 J1/J4 使用镜像限位后，两侧可使用
# 完全反号的关节目标，并得到严格镜像的 TCP 位姿。
LEFT_SAFE_JOINTS = VR_LEFT_JOINTS.copy()
MIRROR_MATRIX = np.diag([1.0, -1.0, 1.0])
TWIST_MIRROR_SIGNS = np.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
JOINT_COMMAND_SPEED = 0.20
JOINT_COMMAND_PERIOD = 0.01
JOINT_MOVE_TOLERANCE = 0.08
JOINT_MOVE_TIMEOUT = 25.0


def select_arm_specs(script_dir):
    left_config = os.path.normpath(
        os.path.join(script_dir, "..", "T701-HT_robot_param", "T701-HT_left.yaml")
    )
    right_config = os.path.normpath(
        os.path.join(script_dir, "..", "T701-HT_robot_param", "T701-HT_right.yaml")
    )
    print("\n请选择机械臂：")
    print("  1. 左臂")
    print("  2. 右臂")
    print("  3. 双臂")
    while True:
        choice = input("请输入 1/2/3: ").strip()
        if choice == "1":
            return [("左臂", left_config, 1.0)]
        if choice == "2":
            return [("右臂", right_config, -1.0)]
        if choice == "3":
            return [
                ("左臂", left_config, 1.0),
                ("右臂", right_config, -1.0),
            ]
        print("输入无效，请输入 1、2 或 3。")


def create_arm_states(openarm_class, script_dir):
    states = []
    for label, config_path, mirror_sign in select_arm_specs(script_dir):
        states.append({
            "label": label,
            "robot": register_arm(openarm_class(config_path), label),
            "mirror_sign": mirror_sign,
        })
    return states


def mirror_translation(vector, mirror_sign):
    vector = np.asarray(vector, dtype=float)
    return vector if mirror_sign > 0.0 else MIRROR_MATRIX @ vector


def mirror_rotation(rotation, mirror_sign):
    rotation = np.asarray(rotation, dtype=float)
    return rotation if mirror_sign > 0.0 else MIRROR_MATRIX @ rotation @ MIRROR_MATRIX


def mirror_twist(twist, mirror_sign):
    twist = np.asarray(twist, dtype=float)
    return twist if mirror_sign > 0.0 else TWIST_MIRROR_SIGNS * twist


def safe_targets(states):
    return {
        state["label"]: LEFT_SAFE_JOINTS * state["mirror_sign"]
        for state in states
    }


def zero_targets(states):
    return {
        state["label"]: np.zeros(state["robot"].motor_count, dtype=float)
        for state in states
    }


def move_selected_joints(states, targets, description, *, control_mode="posvel", kp=None, kd=None):
    """准备和收尾沿用调用脚本的控制模式，不在中途切换模式。"""
    if control_mode not in ("posvel", "mit", "velocity"):
        raise ValueError(f"未知关节移动模式: {control_mode}")
    if control_mode == "mit":
        kp = np.asarray(kp if kp is not None else [30., 50., 60., 25., 15., 10., 10.])
        kd = np.asarray(kd if kd is not None else [3., 5., 6., 2.5, 1.5, 1., 1.])
        if kp.shape != (7,) or kd.shape != (7,) or not np.all(np.isfinite(np.r_[kp, kd])):
            raise ValueError("MIT 准备增益必须是七个有限数值")
    starts = {}
    durations = {}
    for state in states:
        robot = state["robot"]
        label = state["label"]
        target = np.asarray(targets[label], dtype=float)
        if target.shape != (robot.motor_count,) or not np.all(np.isfinite(target)):
            raise ValueError(f"{label}准备姿态无效")
        if np.any(target < robot.joint_limits["lower"]) or np.any(target > robot.joint_limits["upper"]):
            raise ValueError(f"{label}准备姿态超出关节限位")
        starts[label] = np.asarray(robot.get_current_pos(), dtype=float)
        if starts[label].shape != target.shape or not np.all(np.isfinite(starts[label])):
            raise ValueError(f"{label}当前关节位置无效")
        durations[label] = max(3., 1.875 * np.max(abs(target - starts[label])) / JOINT_COMMAND_SPEED)
    print(f"\n{description}")
    deadline = time.monotonic() + JOINT_MOVE_TIMEOUT
    next_deadline = time.perf_counter()
    last_errors = {
        state["label"]: [float("inf")] * state["robot"].motor_count
        for state in states
    }
    reached = False
    move_start = time.monotonic()
    duration = max(durations.values(), default=3.)
    deadline += max(0., duration - JOINT_MOVE_TIMEOUT + 5.)

    while time.monotonic() < deadline:
        reached = True
        for state in states:
            label = state["label"]
            robot = state["robot"]
            target = np.asarray(targets[label], dtype=float)
            if control_mode == "posvel":
                success = robot.Joint_Pos_Vel(
                    target, [JOINT_COMMAND_SPEED] * robot.motor_count,
                    robot.max_torque, iswait=False,
                )
            elif control_mode == "mit":
                u = min(max((time.monotonic() - move_start) / duration, 0.), 1.)
                blend = 10*u**3 - 15*u**4 + 6*u**5
                rate = (30*u**2 - 60*u**3 + 30*u**4) / duration
                delta = target - starts[label]
                q = starts[label] + blend * delta
                velocity = rate * delta
                actual = np.asarray(robot.get_current_pos(), dtype=float)
                if actual.shape != target.shape or not np.all(np.isfinite(actual)):
                    raise ValueError(f"{label}关节反馈无效")
                gravity = np.asarray(robot.get_Gravity(actual), dtype=float)
                if gravity.shape != target.shape or not np.all(np.isfinite(gravity)):
                    raise ValueError(f"{label}重力补偿无效")
                gravity = np.clip(gravity, -np.asarray(robot.max_torque), robot.max_torque)
                success = robot.pos_vel_tqe_kp_kd(q, velocity, gravity, kp, kd)
            else:
                actual = np.asarray(robot.get_current_pos(), dtype=float)
                if actual.shape != target.shape or not np.all(np.isfinite(actual)):
                    raise ValueError(f"{label}关节反馈无效")
                error = target - actual
                velocity = np.clip(2.0 * error, -JOINT_COMMAND_SPEED, JOINT_COMMAND_SPEED)
                velocity[np.abs(error) <= JOINT_MOVE_TOLERANCE] = 0.
                success = robot.Joint_Vel(velocity)
            if not success:
                raise RuntimeError(f"{label}{description}命令未成功下发")
            actual = np.asarray(robot.get_current_pos(), dtype=float)
            if actual.shape != target.shape or not np.all(np.isfinite(actual)):
                raise ValueError(f"{label}关节反馈无效")
            errors = np.abs(actual - target).tolist()
            last_errors[label] = errors
            settled = max(errors) <= JOINT_MOVE_TOLERANCE
            if control_mode == "mit":
                settled = settled and time.monotonic() - move_start >= duration
            reached = reached and settled
        if reached:
            break
        next_deadline += JOINT_COMMAND_PERIOD
        remaining = next_deadline - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_deadline = time.perf_counter()

    for state in states:
        label = state["label"]
        print(f"{label}最大到位误差: {max(last_errors[label]):.3f} rad")
    if not reached:
        details = []
        for state in states:
            label = state["label"]
            values = ", ".join(
                f"J{index + 1}={error:.3f}"
                for index, error in enumerate(last_errors[label])
            )
            details.append(f"{label}[{values}]")
        raise RuntimeError(f"{description}超时: " + "; ".join(details))
    if control_mode == "velocity":
        for state in states:
            if not state["robot"].Joint_Vel(np.zeros(state["robot"].motor_count)):
                raise RuntimeError(f"{state['label']}速度停止命令失败")


def stop_states(states):
    for state in states:
        try:
            state["robot"].set_stop()
        except Exception as error:
            print(f"停止{state['label']}时发生错误: {error}")
