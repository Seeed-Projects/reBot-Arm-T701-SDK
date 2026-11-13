#!/usr/bin/env python3
"""检查 T701-HT 连接并读取状态。 / Check the T701-HT connection and read motor state."""

import argparse
import os
import sys
import time
from pathlib import Path

import yaml

from t701_ht_port_layout import check_serial_layout


SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = SCRIPT_DIR.parent / "T701-HT_robot_param"
ARM_CONFIGS = {
    "left": ("左臂 / Left arm", CONFIG_DIR / "T701-HT_left.yaml"),
    "right": ("右臂 / Right arm", CONFIG_DIR / "T701-HT_right.yaml"),
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


class StateArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        self.print_usage(sys.stderr)
        self.exit(2, f"{self.prog}: 参数错误 / Argument error: {message}\n")


def select_arm():
    print("选择 T701-HT 机械臂 / Select T701-HT arms\n"
          "  1. 左臂 / Left arm\n  2. 右臂 / Right arm\n  3. 双臂 / Both arms")
    while True:
        try:
            choice = input("请输入 1/2/3 / Enter 1/2/3: ").strip().lower()
        except EOFError:
            raise SystemExit("未读取到机械臂选择，请用 --arm 指定。 / No arm selected. Use --arm.") from None
        if choice in ARM_CHOICES:
            return ARM_CHOICES[choice]
        print("输入无效，请输入 1、2 或 3。 / Invalid choice. Enter 1, 2, or 3.")


def check_selected_serial_ports(selected_configs):
    """按 SDK 配置检查串口。 / Check serial order, USB identity, and access against the SDK config."""
    serial_ports = []
    for label, config_path in selected_configs:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        # 原生 SDK 按运行目录解析 param_file。
        # The native SDK resolves param_file from the working directory.
        param_path = Path(config["robot"]["param_file"])
        params = yaml.safe_load(param_path.read_text(encoding="utf-8"))["robot"]
        if params["Serial_Type"] != "/dev/ttyACM":
            raise ValueError(f"{label} — 不支持的串口类型 / Unsupported serial type: {params['Serial_Type']}")
        for board in params["CANboard"].values():
            for port in board["CANport"].values():
                serial_id = port["serial_id"]
                if type(serial_id) is not int or serial_id < 1:
                    raise ValueError(f"{label} — serial_id 必须为正整数 / serial_id must be a positive integer")
                serial_ports.append((label, serial_id))

    check_serial_layout([serial_id for _, serial_id in serial_ports])
    for label, serial_id in serial_ports:
        device = Path(f"/dev/ttyACM{serial_id - 1}")
        if not os.access(device, os.R_OK | os.W_OK):
            raise PermissionError(f"{label} — 串口读写权限不足 / No serial read/write access: {device}")
        print(f"{label} — 串口检查通过 / Serial check passed: {device} (serial_id={serial_id})")


def print_arm_state(label, robot):
    # 沿用原状态示例的四次刷新，减少读到旧缓存的概率。
    # Refresh four times as in the original example to reduce stale readings.
    for _ in range(4):
        robot.send_get_motor_state_cmd()
        robot.motor_send_cmd()

    print(f"\nT701-HT {label} — 状态 / State")
    for index, state in enumerate(robot.get_current_state(), start=1):
        print(
            f"关节 / Joint {index}  位置 / Position {state.position:8.4f} rad  "
            f"速度 / Velocity {state.velocity:8.4f} rad/s  力矩 / Torque {state.torque:8.4f} N·m"
        )
    gripper = robot.get_current_state_gripper()
    print(f"夹爪 / Gripper  位置 / Position {gripper.position:8.4f} rad  "
          f"速度 / Velocity {gripper.velocity:8.4f} rad/s")


def main():
    parser = StateArgumentParser(description=__doc__, add_help=False)
    parser.add_argument("-h", "--help", action="help", help="显示帮助并退出 / Show this help and exit")
    parser.add_argument("--arm", choices=("left", "right", "both"),
                        help="选择左臂、右臂或双臂 / Select the left arm, right arm, or both")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="只检查连接 / Check the connection without opening the port")
    mode.add_argument("--once", action="store_true", help="读取一次后退出 / Read state once and exit")
    args = parser.parse_args()

    selected_arm = args.arm or select_arm()
    selected_configs = ARM_CONFIGS.values() if selected_arm == "both" else [ARM_CONFIGS[selected_arm]]
    # 兼容从项目根目录启动，同时让原生 SDK 按原相对路径加载电机参数。
    # Use the script directory so the native SDK can resolve the motor config paths.
    os.chdir(SCRIPT_DIR)
    try:
        check_selected_serial_ports(selected_configs)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, yaml.YAMLError) as error:
        print(f"T701-HT 连接检查失败 / Connection check failed: {error}", file=sys.stderr)
        return 1
    if args.check:
        return 0

    from Lib.t701_ht import T701HT

    robots = []
    for label, config_path in selected_configs:
        print(f"\n正在连接 / Connecting to T701-HT {label}…")
        robots.append((label, T701HT(str(config_path))))
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
        print("\n已停止读取 T701-HT 状态。 / Stopped reading T701-HT state.")
