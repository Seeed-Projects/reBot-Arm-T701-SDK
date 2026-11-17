"""旧版夹爪姿态到新零位坐标的预设转换；不改变电机命令或真实零位。"""

import numpy as np


J7_LEGACY_OFFSET = np.pi / 2.0


def left_legacy_pose(values):
    """旧左臂位置预设转新坐标；速度、反馈、新录制轨迹不得调用此函数。"""
    pose = np.array(values, dtype=float, copy=True)
    if pose.shape[-1:] != (7,) or not np.all(np.isfinite(pose)):
        raise ValueError("旧姿态必须包含七个有限关节位置")
    pose[..., 6] += J7_LEGACY_OFFSET
    return pose


VR_LEFT_JOINTS = left_legacy_pose([0.3, 0.0, 0.0, -1.87, 0.0, 0.0, 0.0])


def vr_initial_pose(mirrored=False):
    """左右镜像的抬臂初始姿态，夹爪方向与旧版 VR 保持一致。"""
    return -VR_LEFT_JOINTS.copy() if mirrored else VR_LEFT_JOINTS.copy()


def legacy_tool_rotation(rotation):
    """将旧夹爪局部轴的相对旋转转到新末端 frame，不改变世界轴动作。"""
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("旧工具旋转必须是有限的 3x3 矩阵")
    old_to_new = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
    return old_to_new.T @ rotation @ old_to_new
