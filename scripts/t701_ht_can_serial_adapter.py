"""CAN serial adapter: stable USB identity -> current native SDK serial index.

No port is opened during resolution. Native motor behavior is unchanged.
"""
from __future__ import annotations

from contextlib import contextmanager
import copy
import json
import os
from pathlib import Path
import re
import tempfile

import yaml
import hightorque_robot as _native
from hightorque_robot import *  # Preserve the public vendor API.

ROOT = Path(__file__).resolve().parents[1]
BINDINGS = ROOT / 'T701-HT_robot_param/serial_binding.yaml'


def read_yaml(path):
    with Path(path).open(encoding='utf-8') as f:
        result = yaml.safe_load(f)
    if not isinstance(result, dict) or not isinstance(result.get('robot'), dict):
        raise ValueError(f'无效机器人配置: {path}')
    return result


def native_order(prefix, special=False, *, scandir=os.scandir, sys_root=Path('/sys/class/tty')):
    """Match the installed SDK's readdir + reverse + VID/PID filter exactly."""
    prefix = Path(prefix)
    with scandir(prefix.parent) as entries:
        names = [e.name for e in entries if e.name.startswith(prefix.name)]
    result = []
    accepted_vids = {0xfae1} if special else {0xcaf1, 0xcae1}
    for name in reversed(names):
        node = (sys_root / name / 'device').resolve()
        for ancestor in (node, *node.parents):
            if (ancestor / 'idVendor').is_file() and (ancestor / 'idProduct').is_file():
                vid = int((ancestor / 'idVendor').read_text().strip(), 16)
                pid = int((ancestor / 'idProduct').read_text().strip(), 16)
                if vid in accepted_vids and pid == 0xffff:
                    result.append(str(prefix.parent / name))
                break
    return result


def resolve_ports(config, bindings, order, *, by_id=Path('/dev/serial/by-id'), access=os.access):
    """Return a copy with runtime-only serial_id values; never edit source YAML."""
    result = copy.deepcopy(config)
    seen = set()
    rows = []
    serial = bindings['board_serial']
    if not isinstance(serial, str) or not serial or any(c in serial for c in '*?[]/'):
        raise ValueError('serial_binding.yaml 中 board_serial 必须为明确的通信板序列号')
    for board in result['robot']['CANboard'].values():
        for label, port in board['CANport'].items():
            can = port.get('can_port')
            if type(can) is not int or not 1 <= can <= 7:
                raise ValueError(f'{label} 缺少有效的逻辑 can_port (1..7)')
            # This board exposes seven interfaces: if00, if02, ..., if0c.
            pattern = f'*_{serial}-if{2*(can-1):02x}'
            links = list(by_id.glob(pattern))
            if len(links) != 1:
                raise RuntimeError(f'CAN{can}: 稳定 USB 接口缺失或不唯一 ({pattern})；检查通信板连接')
            device = str(links[0].resolve(strict=True))
            if device not in order:
                raise RuntimeError(f'CAN{can}: {device} 不在 SDK 可用串口中: {order}')
            if device in seen:
                raise ValueError(f'重复分配设备 {device}')
            if not access(device, os.R_OK | os.W_OK):
                raise PermissionError(f'{device} 无读写权限；在当前终端执行 newgrp dialout 后重新激活 panthera')
            seen.add(device)
            port['serial_id'] = order.index(device) + 1
            rows.append({'can_port': can, 'device': device, 'serial_id': port['serial_id'], 'stable_path': str(links[0])})
    return result, rows


@contextmanager
def prepared_config(config_path, *, bindings_path=BINDINGS):
    original_path = Path(config_path).expanduser().resolve(strict=True)
    original = read_yaml(original_path)
    source = original['robot'].get('param_file')
    motor = read_yaml(original_path.parent / source) if source else original
    ports = [p for b in motor['robot']['CANboard'].values() for p in b['CANport'].values()]
    if not ports or any('can_port' not in p for p in ports):
        raise ValueError('电机配置缺少 can_port；请填写逻辑 CAN 口，不再用 serial_id 表示接线')
    bindings = yaml.safe_load(Path(bindings_path).read_text(encoding='utf-8'))
    # Always enumerate the same prefix as the native SDK.
    prefix = motor['robot']['Serial_Type']
    special = motor['robot'].get('board_special_flag', False)
    order = native_order(prefix, special)
    resolved, rows = resolve_ports(motor, bindings, order)
    # The native constructor always reads robot.name, including flat configs.
    resolved['robot']['name'] = original['robot'].get('name', original['robot'].get('robot_name', 'T701-HT'))
    resolved['robot'].pop('param_file', None)
    if native_order(prefix, special) != order:
        raise RuntimeError('解析期间串口列表发生变化，请等待设备稳定后重新启动')
    for row in rows:
        print(f"[串口自适应] CAN{row['can_port']} -> {row['device']} -> serial_id={row['serial_id']}", flush=True)
    with tempfile.TemporaryDirectory(prefix='openarm_serial_') as directory:
        path = Path(directory) / 'motor_runtime.yaml'
        path.write_text(yaml.safe_dump(resolved, allow_unicode=True, sort_keys=False), encoding='utf-8')
        yield path, rows


class Robot(_native.Robot):
    def __init__(self, config_path=None):
        if config_path is None:
            raise ValueError('必须明确指定项目配置路径')
        with prepared_config(config_path) as (path, rows):
            super().__init__(str(path))
        self.serial_mapping = rows


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='只解析串口映射，不创建 Robot、不打开串口')
    parser.add_argument('configs', nargs='+')
    args = parser.parse_args()
    for config in args.configs:
        with prepared_config(config) as (_, rows):
            print(json.dumps(rows, ensure_ascii=False, indent=2))
