# T701-HT SDK

[English](README.md)

Ubuntu 22.04（x86_64），Python 3.10。以下命令在项目根目录运行。

## 1. 安装环境

未安装 Conda 时，先安装 Miniforge。

```bash
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

按提示初始化 Conda，重新打开终端并回到项目根目录。

```bash
conda create -n t701-ht python=3.10 pip
conda activate t701-ht
python -I -m pip install ./motor_whl/hightorque_robot-1.2.0-cp310-cp310-linux_x86_64.whl
python -I -m pip install -r requirements.txt
```

验证依赖和 URDF 模型（无需连接机械臂）。

```bash
python -I -c "import hightorque_robot, numpy, pinocchio, scipy, yaml; pinocchio.buildModelFromUrdf('T701-HT_URDF/urdf/T701-HT.urdf'); print('T701-HT 环境检查通过')"
```

## 2. 检查连接

1. 接上电源。

   ![电源接线图](images/power-connection.jpg)

2. 按下圆形开关开机。
3. 用 USB 数据线连接电脑和主板的 Type-C 接口。

   ![主板 Type-C 接线图](images/type-c-mainboard.jpg)

双臂用 `--arm both`，单臂用 `--arm left` 或 `--arm right`。

```bash
ls -l /dev/ttyACM*
python scripts/t701_ht_read_state.py --arm both --check
```

默认左臂为 `/dev/ttyACM0`，右臂为 `/dev/ttyACM1`。若权限不足，用以下命令临时允许所有本机用户读写串口，再运行检查。单臂只填对应串口。拔插或重启后需重新授权。

```bash
sudo chmod 666 /dev/ttyACM0 /dev/ttyACM1
```

## 3. 读取状态

交互模式中，`1` 读左臂，`2` 读右臂，`3` 读双臂。按 `Ctrl+C` 退出。

```bash
python scripts/t701_ht_read_state.py
```

读取一次后退出。

```bash
python scripts/t701_ht_read_state.py --arm both --once
```

## 4. 设置零点

将所选机械臂的关节和夹爪摆到零位，运行脚本并输入 `yes` 确认覆盖原零点。将 `left` 换为 `right` 或 `both` 可选择右臂或双臂。

```bash
python scripts/t701_ht_set_zero.py --arm left
```

## 5. 运行示例

确认关节零点，取下负载并清空机械臂的活动范围。首次运行时选单臂。三个脚本都通过 `1`、`2`、`3` 选择左臂、右臂或双臂，也支持 `--arm left`、`--arm right` 或 `--arm both`。

### 重力补偿

示例以 MIT 力矩前馈施加 `0.8 × G(q)`，减轻手动移动机械臂时的负担。关节 `Kp`、`Kd` 和夹爪力矩均为零。启动和退出时托住机械臂。

```bash
python scripts/t701_ht_gravity_compensation.py
```

### 关节阻抗

机械臂启动后会向固定目标姿态移动。左臂目标为 `[-0.2, -0.2, 0.2, -0.4, 0.5, 0.2, -0.1]` rad，右臂逐关节取反。

关节阻抗 + 重力前馈

```bash
python scripts/t701_ht_joint_impedance_with_gravity.py
```

关节阻抗 + 重力前馈 + 摩擦补偿

```bash
python scripts/t701_ht_joint_impedance_with_gravity_friction.py
```

三个示例都显示关节指令力矩（N·m），按配置限幅，并设置 250 ms 电机命令超时。按 `Ctrl+C` 向所选机械臂和夹爪发送停止命令。反馈过期、电机故障或关节越限也会触发停止。退出时托住机械臂。

许可见 [LICENSE](LICENSE)。
