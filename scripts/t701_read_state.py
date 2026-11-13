#!/usr/bin/env python3
"""检查 T701 通信连接并读取关节和夹爪状态。"""

import argparse
import os
import sys
import time
from pathlib import Path

import yaml

from openarm_port_layout import check_serial_layout


SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = SCRIPT_DIR.parent / "T701_robot_param"
ARM_CONFIGS = {
    "left": ("左臂", CONFIG_DIR / "T701_left.yaml"),
    "right": ("右臂", CONFIG_DIR / "T701_right.yaml"),
}
ARM_CHOICES = {
    "1": "left",
    "2": "right",
    "3": "both",
    "left": "left",
    "right": "right",
    "both": "both",
    "左": "left",
    "右": "right",
    "双": "both",
}


def select_arm():
    print("请选择 T701 机械臂：\n  1. 左臂\n  2. 右臂\n  3. 双臂")
    while True:
        try:
            choice = input("请输入 1/2/3：").strip().lower()
        except EOFError:
            raise SystemExit("未读取到机械臂选择，可用 --arm 指定。") from None
        if choice in ARM_CHOICES:
            return ARM_CHOICES[choice]
        print("输入无效，请输入 1、2 或 3。")


def check_selected_serial_ports(selected_configs):
    """按原 SDK 配置检查设备枚举、USB 标识和读写权限。"""
    serial_ports = []
    for label, config_path in selected_configs:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        # 原生 SDK 按运行目录解析 param_file。
        param_path = Path(config["robot"]["param_file"])
        params = yaml.safe_load(param_path.read_text(encoding="utf-8"))["robot"]
        if params["Serial_Type"] != "/dev/ttyACM":
            raise ValueError(f"{label}不支持当前串口检查：{params['Serial_Type']}")
        for board in params["CANboard"].values():
            for port in board["CANport"].values():
                serial_id = port["serial_id"]
                if type(serial_id) is not int or serial_id < 1:
                    raise ValueError(f"{label} serial_id 必须为从 1 开始的整数")
                serial_ports.append((label, serial_id))

    check_serial_layout([serial_id for _, serial_id in serial_ports])
    for label, serial_id in serial_ports:
        device = Path(f"/dev/ttyACM{serial_id - 1}")
        if not os.access(device, os.R_OK | os.W_OK):
            raise PermissionError(f"{label}没有串口读写权限：{device}")
        print(f"{label}串口检查通过：{device}（serial_id={serial_id}）")


def print_arm_state(label, robot):
    # 沿用原状态示例的四次刷新，减少读到旧缓存的概率。
    for _ in range(4):
        robot.send_get_motor_state_cmd()
        robot.motor_send_cmd()

    print(f"\nT701 {label}状态")
    for index, state in enumerate(robot.get_current_state(), start=1):
        print(
            f"关节 {index}  位置 {state.position:8.4f} rad  "
            f"速度 {state.velocity:8.4f} rad/s  力矩 {state.torque:8.4f} N·m"
        )
    gripper = robot.get_current_state_gripper()
    print(f"夹爪  位置 {gripper.position:8.4f} rad  速度 {gripper.velocity:8.4f} rad/s")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", choices=("left", "right", "both"), help="选择左臂、右臂或双臂")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="只检查连接，不打开串口")
    mode.add_argument("--once", action="store_true", help="读取一次状态后退出")
    args = parser.parse_args()

    selected_arm = args.arm or select_arm()
    selected_configs = ARM_CONFIGS.values() if selected_arm == "both" else [ARM_CONFIGS[selected_arm]]
    # 兼容从项目根目录启动，同时让原生 SDK 按原相对路径加载电机参数。
    os.chdir(SCRIPT_DIR)
    try:
        check_selected_serial_ports(selected_configs)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, yaml.YAMLError) as error:
        print(f"T701 连接检查失败：{error}", file=sys.stderr)
        return 1
    if args.check:
        return 0

    from Lib.Openarm import Openarm

    robots = []
    for label, config_path in selected_configs:
        print(f"\n正在连接 T701 {label}…")
        robots.append((label, Openarm(str(config_path))))
        time.sleep(0.5)

    while True:
        for label, robot in robots:
            print_arm_state(label, robot)
        if args.once:
            return 0
        time.sleep(0.5)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n已停止读取 T701 状态。")
