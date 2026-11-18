"""Require an explicit channel/motor instead of a missing legacy motor_1.yaml."""
from contextlib import contextmanager
from pathlib import Path
import argparse
import tempfile
import yaml

@contextmanager
def selected_motor_config():
    parser=argparse.ArgumentParser(description='单电机示例：明确选择逻辑 CAN 口和电机 ID')
    parser.add_argument('--can-port',type=int,choices=range(1,8),required=True)
    parser.add_argument('--motor-id',type=int,choices=range(1,9),required=True)
    parser.add_argument('--arm',choices=('left','right'),required=True,help='选择对应电机型号配置')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    config=yaml.safe_load((root/f'T701-HT_robot_param/motor_param/7dof_T701-HT_params_{args.arm}.yaml').read_text())
    robot=config['robot'];robot['name']=robot['robot_name']='t701_ht_single_motor_example'
    robot['home_arm']=args.arm
    board=robot['CANboard']['No_1_CANboard'];board['CANport_num']=1
    port=board['CANport']['CANport_1'];port['can_port']=args.can_port;port['motor_num']=1
    matches=[m for m in port['motor'].values() if m['id']==args.motor_id]
    assert len(matches)==1
    port['motor']={'motor1':matches[0]}
    with tempfile.TemporaryDirectory(prefix='t701_ht_single_motor_') as directory:
        path=Path(directory)/'motor.yaml';path.write_text(yaml.safe_dump(config))
        yield str(path)


@contextmanager
def managed_motor_motion():
    """底层运动示例共用收尾；纯状态与标零示例不调用此入口。"""
    import t701_ht_can_serial_adapter as htr
    from Lib.t701_ht import ShutdownSignals, raw_motor_home_and_finish

    robot = None
    signals = ShutdownSignals().install()
    try:
        with selected_motor_config() as config_path:
            config = yaml.safe_load(Path(config_path).read_text())
            project = Path(__file__).resolve().parents[2]
            side = config["robot"].get("home_arm")
            if side not in ("left", "right"):
                raise RuntimeError("底层示例未指定回零使用的机械臂配置")
            config["robot"]["home_parameters"] = yaml.safe_load((project / f"T701-HT_robot_param/T701-HT_{side}.yaml").read_text())["robot"]
            robot = htr.Robot(config_path)
            try:
                yield robot
            except KeyboardInterrupt:
                print("\n运动已中断，进入回零收尾。", flush=True)
    finally:
        try:
            if robot is not None:
                raw_motor_home_and_finish(robot, config, signals=signals)
        finally:
            signals.restore()
