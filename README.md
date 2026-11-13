# T701-HT SDK

本项目提供 T701-HT 的环境安装、连接检查和状态读取示例。

## 1. 安装环境

本指南适用于 Ubuntu 22.04（x86_64），使用 Python 3.10。先在终端进入 SDK 项目根目录。

### 创建 Python 环境

如果电脑还没有 Conda，从 [Miniforge 官方下载页](https://conda-forge.org/download/) 下载 Linux x86_64 安装程序。在下载目录用 `bash` 运行安装程序，按提示初始化 Conda，然后重新打开终端并回到项目根目录。

```bash
conda create -n t701-ht python=3.10 pip
conda activate t701-ht
uname -m
python --version
```

确认 `uname -m` 输出为 `x86_64`，`python --version` 显示 Python 3.10，然后安装底层 SDK 和状态读取依赖。

```bash
python -m pip install ./motor_whl/hightorque_robot-1.2.0-cp310-cp310-linux_x86_64.whl
python -m pip install -r requirements.txt
```

### 验证安装

这条命令会导入依赖并验证本地模型可以加载，无需连接机械臂。

```bash
python -c "import hightorque_robot, numpy, pinocchio, scipy, yaml; pinocchio.buildModelFromUrdf('T701-HT_URDF/urdf/T701-HT.urdf'); print('T701-HT 环境检查通过')"
```

## 2. 检查连接

给通信板供电，将 USB 数据线接到运行 SDK 的 Ubuntu 电脑。在项目根目录查看串口并检查所选机械臂。双臂用 `both`，只接一侧时用 `left` 或 `right`。

```bash
ls -l /dev/ttyACM*
python scripts/t701_ht_read_state.py --arm both --check
```

脚本按原配置检查串口枚举顺序、USB 标识和当前账户的读写权限。原配置预期左臂 `serial_id=1` 对应 `/dev/ttyACM0`，右臂 `serial_id=2` 对应 `/dev/ttyACM1`。这一步只检查主机端设备信息，实际电机通信要通过下一步的状态反馈确认。

- 若没有列出设备，先检查通信板供电和 USB 数据线。
- 若报告枚举不符，按提示核对两臂的接线及设备顺序，并检查是否有其他 `ttyACM` 设备。
- 若报告权限不足，查看 `ls -l` 显示的设备所属组。Ubuntu 上如果设备组为 `dialout`，管理员可执行 `sudo usermod -aG dialout "$USER"`，当前账户重新登录后再检查。

## 3. 读取状态

进入交互菜单后，输入 `1` 读左臂、`2` 读右臂、`3` 读双臂。脚本会先执行连接检查，然后约每 0.5 秒刷新一轮。按 `Ctrl+C` 退出。

```bash
python scripts/t701_ht_read_state.py
```

也可指定机械臂并只读取一轮。

```bash
python scripts/t701_ht_read_state.py --arm both --once
```

输出包含每个关节的位置（rad）、速度（rad/s）、力矩（N·m），以及夹爪的位置和速度。请按实际连接核对左右臂反馈。脚本沿用原 SDK 的状态查询与刷新方式，读数来自 SDK 缓存。

## 项目来源

状态读取和连接检查沿用原 SDK 的实现。左右臂配置在 `T701-HT_robot_param/`，模型在 `T701-HT_URDF/urdf/T701-HT.urdf`。底层包和硬件参数沿用原 SDK。原许可见 [LICENSE](LICENSE)。
