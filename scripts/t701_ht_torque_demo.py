"""T701-HT 力矩示例共用的反馈检查和控制循环。 / Feedback checks and control loop for T701-HT torque demos."""

import os
import sys
import time

import numpy as np
import yaml

from t701_ht_read_state import (
    ARM_CONFIGS,
    SCRIPT_DIR,
    StateArgumentParser,
    check_selected_serial_ports,
    select_arm,
)


COMMAND_INTERVAL = 0.006  # 为每个串口留出至少 5 ms 的发送间隔。 / Allow at least 5 ms between sends per port.
FEEDBACK_TIMEOUT = 0.25
MOTOR_TIMEOUT_MS = 250


def read_joint_feedback(label, robot):
    states = robot.get_current_state()
    now = time.time()
    for index, state in enumerate(states, start=1):
        if state.fault:
            raise RuntimeError(f"{label} J{index} — 电机故障 / Motor fault: 0x{state.fault:02X}")
        if not (-0.01 <= now - state.time <= FEEDBACK_TIMEOUT) or not np.isfinite(state.position):
            raise RuntimeError(f"{label} J{index} — 反馈过期或无效 / Stale or invalid feedback")

    position = np.array([state.position for state in states])
    if np.any(position < robot.joint_limits["lower"]) or np.any(position > robot.joint_limits["upper"]):
        raise RuntimeError(f"{label} — 当前位置超出关节限位 / Current position exceeds joint limits")

    return position, states


def torque_demo_parser(description):
    parser = StateArgumentParser(description=description, add_help=False)
    parser.add_argument("-h", "--help", action="help", help="显示帮助并退出 / Show this help and exit")
    parser.add_argument("--arm", choices=("left", "right", "both"),
                        help="选择左臂、右臂或双臂 / Select the left arm, right arm, or both")
    return parser


def run_torque_demo(arm, read_torque, name):
    selected_arm = arm or select_arm()
    sides = tuple(ARM_CONFIGS) if selected_arm == "both" else (selected_arm,)
    selected_configs = [ARM_CONFIGS[side] for side in sides]
    # 底层 SDK 按运行目录解析电机参数。 / The native SDK resolves motor parameters from the working directory.
    os.chdir(SCRIPT_DIR)
    robots = []
    exit_code = 0
    try:
        check_selected_serial_ports(selected_configs)

        from Lib.t701_ht import T701HT

        for side, (label, config_path) in zip(sides, selected_configs):
            print(f"正在连接 T701-HT {label} / Connecting…", flush=True)
            robot = T701HT(str(config_path))
            robots.append((side, label, robot))
            if [motor.get_motor_id() for motor in robot.Motors] != list(range(1, 9)):
                raise RuntimeError(f"{label} — 电机 ID 顺序应为 1–8 / Expected motor IDs 1–8 in order")
            robot.set_timeout(MOTOR_TIMEOUT_MS)

        for _ in range(4):
            for _, _, robot in robots:
                robot.send_get_motor_state_cmd()
            time.sleep(0.02)

        print(f"{name} — 按 Ctrl+C 退出 / Press Ctrl+C to exit.", flush=True)
        next_log = 0.0
        while True:
            # 双臂都通过反馈检查后，再发送本轮力矩。 / Validate both arms before sending any torque for the cycle.
            commands = []
            for side, label, robot in robots:
                position, states = read_joint_feedback(label, robot)
                torque = read_torque(side, robot, position, states)
                if not np.all(np.isfinite(torque)):
                    raise RuntimeError(f"{label} — 关节力矩无效 / Invalid joint torque")
                commands.append((position, np.clip(torque, -robot.max_torque, robot.max_torque)))
            for (_, label, robot), (position, torque) in zip(robots, commands):
                zeros = [0.0] * robot.motor_count
                if not robot.gripper_control_MIT(0, 0, 0, 0, 0, send=False):
                    raise RuntimeError(f"{label} — 夹爪命令被拒绝 / Gripper command rejected")
                if not robot.pos_vel_tqe_kp_kd(position, zeros, torque, zeros, zeros):
                    raise RuntimeError(f"{label} — 力矩命令被拒绝 / Torque command rejected")

            now = time.monotonic()
            if now >= next_log:
                for (_, label, _), (_, torque) in zip(robots, commands):
                    print(f"{label} — 关节力矩 / Joint torque (N·m): {np.round(torque, 3)}", flush=True)
                next_log = now + 0.5

            time.sleep(COMMAND_INTERVAL)
            for _, _, robot in robots:
                robot.send_get_motor_state_cmd()
            time.sleep(COMMAND_INTERVAL)
    except KeyboardInterrupt:
        exit_code = 130
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, yaml.YAMLError) as error:
        print(f"{name} — 控制中断 / Control interrupted: {error}", file=sys.stderr)
        exit_code = 1
    finally:
        if robots:
            time.sleep(COMMAND_INTERVAL)
        for _, label, robot in robots:
            try:
                robot.set_stop()
                print(f"{label} — 已发送停止命令 / Stop command sent", flush=True)
            except Exception as error:
                print(f"{label} — 停止命令发送失败 / Failed to send stop command: {error}", file=sys.stderr)
                exit_code = 1
    return exit_code
