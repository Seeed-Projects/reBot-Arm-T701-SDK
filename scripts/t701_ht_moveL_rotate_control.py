#!/usr/bin/env python3
"""左臂、右臂或双臂：VR 初始姿态下的五秒连续末端旋转演示。"""

import os
import time

import numpy as np
from scipy.interpolate import PPoly, make_interp_spline
from scipy.spatial.transform import Rotation

from Lib import T701HT
from Lib.t701_ht import legacy_tool_rotation, vr_initial_pose
from Lib.t701_ht import register_arm, return_home_on_exit
from Lib.keyboard_arm_utils import move_selected_joints, select_arm_specs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


DEMO_DURATION = 5.0
CONTROL_PERIOD = 0.01
DEMO_KP = np.array([20., 30., 15., 15., 10., 8., 5.])
# 保持原刚度，阻尼相对原 moveL 参数减小 20%。
DEMO_KD = np.array([.8, 1.04, .48, .48, .48, .48, .16])


def smooth_spline(times, values):
    """五次样条贯穿中间点；仅整个动作的首尾速度、加速度为零。"""
    values = np.asarray(values, dtype=float)
    zeros = np.zeros(values.shape[1:])
    return make_interp_spline(
        times, values, k=5,
        bc_type=([(1, zeros), (2, zeros)], [(1, zeros), (2, zeros)]),
    )


def trajectory_extrema(spline, order=0):
    """检查每个分段多项式的全部极值，包含采样点之间的样条极值。"""
    lower, upper = [], []
    for joint in range(spline.c.shape[1]):
        piecewise = PPoly.from_spline((spline.t, spline.c[:, joint], spline.k))
        if order:
            piecewise = piecewise.derivative(order)
        roots = piecewise.derivative().roots(extrapolate=False)
        roots = roots[np.isfinite(roots) & (roots > 0.) & (roots < DEMO_DURATION)]
        samples = np.r_[0., spline.t, roots, DEMO_DURATION]
        samples = samples[(samples >= 0.) & (samples <= DEMO_DURATION)]
        values = piecewise(samples)
        lower.append(np.min(values))
        upper.append(np.max(values))
    return np.asarray(lower), np.asarray(upper)


def plan_demo(robots, pose_at, reference_targets):
    """运动前完成逆解和极值检查；幅度适配五秒时长，不延长演示时间。"""
    sample_times = np.linspace(0., DEMO_DURATION, 101)
    scale = 1.
    for _ in range(8):
        plans = {}
        largest_ratio = 1.
        for label, robot, mirror_sign in robots:
            reference = np.asarray(reference_targets[label], dtype=float)
            q = reference.copy()
            positions = [q.copy()]
            for sample in sample_times[1:]:
                position, rotation = pose_at(label, mirror_sign, sample, scale)
                solution = robot.inverse_kinematics(
                    target_position=position, target_rotation=rotation,
                    init_q=q, multi_init=False, max_iter=1000,
                    eps=1e-8, damping=1e-3,
                )
                if solution is None:
                    raise RuntimeError(f"{label}五秒路径逆解失败（{sample:.2f} 秒）")
                q = np.asarray(solution, dtype=float)
                if q.shape != reference.shape or not np.all(np.isfinite(q)):
                    raise RuntimeError(f"{label}五秒路径逆解无效")
                if np.max(abs(q - positions[-1])) > robot.jump_threshold:
                    raise RuntimeError(f"{label}五秒路径存在关节跳变")
                positions.append(q.copy())
            spline = smooth_spline(sample_times, positions)
            lower, upper = trajectory_extrema(spline)
            if (np.any(lower < robot.joint_limits["lower"] - 1e-9)
                    or np.any(upper > robot.joint_limits["upper"] + 1e-9)):
                raise RuntimeError(f"{label}五秒路径超出关节限位")
            velocity_low, velocity_high = trajectory_extrema(spline, 1)
            accel_low, accel_high = trajectory_extrema(spline, 2)
            velocity_peak = np.maximum(abs(velocity_low), abs(velocity_high))
            accel_peak = np.maximum(abs(accel_low), abs(accel_high))
            largest_ratio = max(
                largest_ratio,
                float(np.max(velocity_peak / robot.velocity_limits)),
                float(np.max(accel_peak / robot.acceleration_limits)),
            )
            plans[label] = spline
        if largest_ratio <= 1.000001:
            print(f"连续演示共 {DEMO_DURATION:.0f} 秒；移动到 VR 初始姿态及最终回零另计。")
            if scale < .999:
                print(f"保持五秒完成，动作幅度适配为原演示的 {scale * 100:.0f}%。")
            return plans
        scale *= min(.9, .95 / largest_ratio)
        if scale < .05:
            break
    raise RuntimeError("无法在五秒内生成连续路径")


def execute_demo(robots, plans):
    """左右臂共用一个时钟，实时采样样条；不堆积过期轨迹点。"""
    print("开始五秒连续演示（全程 MIT 控制）...", flush=True)
    start = time.perf_counter()
    tick = 0
    while True:
        elapsed = min(time.perf_counter() - start, DEMO_DURATION)
        for label, robot, _ in robots:
            q = plans[label](elapsed)
            velocity = plans[label](elapsed, 1)
            gravity = np.asarray(robot.get_Gravity(q), dtype=float)
            if gravity.shape != q.shape or not np.all(np.isfinite(gravity)):
                raise RuntimeError(f"{label}重力前馈无效")
            if robot.pos_vel_tqe_kp_kd(
                q, velocity,
                np.clip(gravity, -np.asarray(robot.max_torque), robot.max_torque),
                DEMO_KP, DEMO_KD,
            ) is False:
                raise RuntimeError(f"{label}五秒路径指令发送失败")
        if elapsed >= DEMO_DURATION:
            break
        tick = max(tick + 1, int((time.perf_counter() - start) / CONTROL_PERIOD) + 1)
        deadline = start + min(tick * CONTROL_PERIOD, DEMO_DURATION)
        remaining = deadline - time.perf_counter()
        if remaining > 0.:
            time.sleep(remaining)
    print(f"五秒连续演示完成，实际 {time.perf_counter() - start:.3f} 秒。", flush=True)



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
