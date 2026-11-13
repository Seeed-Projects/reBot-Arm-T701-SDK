"""两块七接口 USB 通信板：SDK serial_id 从 1 开始，ttyACM 从 0 开始。"""
import os
from pathlib import Path


def port_description(serial_id):
    board, port = divmod(serial_id - 1, 7)
    return f"板{board + 1}/CAN{port + 1}（ttyACM{serial_id - 1}，serial_id={serial_id}）"


def check_serial_layout(serial_ids):
    """在 SDK 打开串口前核对其实际枚举顺序，拒绝缺板或移位连接。

    当前 SDK 使用 readdir 后倒序的枚举，不是字典序排序；ACM10 不应
    排在 ACM2 前。这里只读目录/sysfs，不打开串口，也不构造 Robot。
    """
    ports = [entry.name for entry in os.scandir('/dev') if entry.name.startswith('ttyACM')][::-1]
    for serial_id in serial_ids:
        expected = f'ttyACM{serial_id - 1}'
        if len(ports) < serial_id or ports[serial_id - 1] != expected:
            raise RuntimeError(f"串口枚举与配置不符：需要 {port_description(serial_id)}；当前顺序={ports}")
        device = (Path('/sys/class/tty') / expected / 'device').resolve()
        usb = device.parent
        try:
            vendor = (usb / 'idVendor').read_text().strip().lower()
            product = (usb / 'idProduct').read_text().strip().lower()
        except OSError as error:
            raise RuntimeError(f"无法核对 {expected} 的通信板身份") from error
        if (vendor, product) != ('caf1', 'ffff'):
            raise RuntimeError(f"{expected} 不是已验证的通信板：USB {vendor}:{product}")
