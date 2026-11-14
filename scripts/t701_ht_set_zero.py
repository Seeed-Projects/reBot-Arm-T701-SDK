#!/usr/bin/env python3
"""设置 T701-HT 零点。 / Set the T701-HT zero position."""

import os
import sys
import time

import yaml

from t701_ht_read_state import (
    ARM_CONFIGS,
    SCRIPT_DIR,
    StateArgumentParser,
    check_selected_serial_ports,
    print_arm_state,
    select_arm,
)


def main():
    parser = StateArgumentParser(description=__doc__, add_help=False)
    parser.add_argument("-h", "--help", action="help", help="显示帮助并退出 / Show this help and exit")
    parser.add_argument("--arm", choices=("left", "right", "both"),
                        help="选择左臂、右臂或双臂 / Select the left arm, right arm, or both")
    args = parser.parse_args()

    selected_arm = args.arm or select_arm()
    selected_configs = ARM_CONFIGS.values() if selected_arm == "both" else [ARM_CONFIGS[selected_arm]]
    # 底层 SDK 按运行目录解析电机参数。 / The native SDK resolves motor parameters from the working directory.
    os.chdir(SCRIPT_DIR)
    try:
        check_selected_serial_ports(selected_configs)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, yaml.YAMLError) as error:
        print(f"T701-HT 连接检查失败 / Connection check failed: {error}", file=sys.stderr)
        return 1

    labels = " + ".join(label for label, _ in selected_configs)
    print(f"T701-HT 设零对象 / Zeroing target: {labels}")
    try:
        confirmation = input(
            "以当前关节和夹爪位置覆盖原零点，输入 yes 确认 / "
            "Type yes to overwrite the zero positions with the current joint and gripper positions: "
        ).strip().lower()
    except EOFError:
        confirmation = ""
    if confirmation != "yes":
        print("已取消设零。 / Zeroing canceled.")
        return 1

    from Lib.t701_ht import T701HT

    # 所选机械臂全部初始化后再设零。 / Initialize every selected arm before zeroing.
    robots = [(label, T701HT(str(config_path))) for label, config_path in selected_configs]
    for label, robot in robots:
        print(f"正在设置 T701-HT {label} 零点 / Sending zero command…", flush=True)
        robot.set_reset_zero()
        robot.motor_send_cmd()
        print(f"T701-HT {label} 设零命令已发送 / Zero command sent.", flush=True)

    time.sleep(1)
    for label, robot in robots:
        print_arm_state(label, robot)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nT701-HT 设零流程已中断。 / T701-HT zeroing interrupted.", file=sys.stderr)
        raise SystemExit(130) from None
