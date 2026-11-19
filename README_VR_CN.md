# T701-HT VR 遥操作

[English](README_VR.md)

通过 VR 左右手柄控制对应机械臂，支持末端位姿跟随和夹爪控制。本 Demo 已在 Quest 上测试；Pico 用户可通过支持 WebXR 的头显浏览器，使用相同的 WebXR → WebSocket → UDP 链路接入。

## 1. 准备环境

先完成 [SDK 环境与机械臂通信配置](README_CN.md)，检查反馈和零点。以下命令从 SDK 根目录执行，每个新终端先激活环境：

```bash
conda activate t701-ht
```

## 2. 启动串流

```bash
python Quest_controller_stream/quest_stream_tui.py
```

方向键选择字段，左右键切换值，`E` 编辑地址或端口，`Enter` 执行按钮。切换连接方式前先 **Stop Server**，再按所选方式设置参数：

| 设置 | USB 有线 | WiFi 无线 |
| --- | --- | --- |
| Mode | USB | WiFi |
| Force HTTPS | False | True |
| HTTP Host / Port | 0.0.0.0 / 8080 | 0.0.0.0 / 8080 |
| WS Host / Port | 0.0.0.0 / 8765 | 0.0.0.0 / 8765 |
| UDP IP / Port | 127.0.0.1 / 5005 | 127.0.0.1 / 5005 |
| Verify Port | 5005 | 5005 |

### USB 有线

头显开启开发者模式，用 USB 数据线连接电脑，并在头显内允许 USB 调试。Ubuntu 未安装 ADB 时执行 `sudo apt install adb`，然后在另一终端检查：

```bash
adb devices
```

设备状态应为 `device`；`unauthorized` 表示需在头显内确认授权。

选择 **Start Server**，程序通过 `adb reverse` 转发 8080、8765 端口。在头显浏览器打开：

```text
http://localhost:8080/index.html?ws=ws://localhost:8765
```

网页 WebSocket 地址设为 `ws://localhost:8765`。USB 使用 HTTP / WS，无需 HTTPS 证书。

### WiFi 无线

头显与电脑连接同一局域网，无需 USB 数据线或 ADB。电脑需安装 OpenSSL；Ubuntu 可执行 `sudo apt install openssl`。查看电脑实际局域网 IP，不要选择 Docker 或 VPN 网卡地址：

```bash
hostname -I
```

保留证书路径 `.certs/quest-controller.crt` 和私钥路径 `.certs/quest-controller.key`，选择 **Start Server**。WiFi 使用 HTTPS / WSS，证书文件缺失时自动生成自签名证书。

在头显浏览器打开界面的 WiFi URL。例如电脑 IP 为 `192.168.1.100` 时：

```text
https://192.168.1.100:8080/index.html?ws=wss://192.168.1.100:8765
```

替换为电脑实际 IP，首次访问按浏览器提示处理证书并重新加载。WebSocket 地址应为 `wss://电脑实际IP:8765`；`0.0.0.0` 是监听地址，不能作为访问地址。串流与控制程序在同一电脑运行时，UDP 仍为 `127.0.0.1:5005`。

### 检查连接

按所选方式检查网页的 WebSocket 地址，避免使用浏览器保存的另一种连接设置。点击 **Start XR Stream**，确认 WebSocket 为 `Connected`、XR 为 `Running`，左右手柄为 `Tracking`。

在电脑选择 **Start Verify**，确认包数和手柄数据变化后选择 **Stop Verify**。保持 Server、所选连接和头显 XR 串流运行。

## 3. 启动双臂控制

**先停止 Verify 和其他 UDP 5005 接收程序。收到数据后，机械臂会先左臂、再右臂移动到 VR 初始姿态，启动前清空活动范围和回零路径。**

另开终端，从 SDK 根目录执行：

```bash
conda activate t701-ht
cd scripts
python t701_ht_vr_dual_cartesian_qp_control.py
```

该入口直接控制双臂。左臂使用 CAN1、右臂使用 CAN2，对应配置为 `T701-HT_robot_param/T701-HT_left.yaml` 和 `T701-HT_right.yaml`。

- 按住对应手柄 **Squeeze 握持键**控制该侧机械臂，松开后保持当前位置。
- 按住 Squeeze 时，**Trigger 扳机**控制对应夹爪。
- 手柄主按键（程序提示为 A 键）使对应臂平滑复位到本次初始关节角。
- `Ctrl+C` 退出控制，机械臂以 MIT 模式回零并停止；等待退出完成。串流界面按 `Q` 停止服务并退出。

端口被占用或启动后 10 秒内收不到数据时，程序在初始化机械臂前退出。检查 Verify 已停止、头显 XR 串流已开启，UDP 目标为 `127.0.0.1:5005`。

## 其他入口

不使用终端界面时，从 SDK 根目录选择一条命令。不要与界面服务同时运行：

```bash
# USB：ADB 转发，HTTP / WS
python Quest_controller_stream/start_stream_server.py --http-port 8080 --ws-port 8765 --udp-ip 127.0.0.1 --udp-port 5005 --adb auto

# WiFi：关闭 ADB，自动启用 HTTPS / WSS
python Quest_controller_stream/start_stream_server.py --http-port 8080 --ws-port 8765 --udp-ip 127.0.0.1 --udp-port 5005 --adb never
```

桌面界面 `Quest_controller_stream/quest_stream_ui.py` 需额外安装 `PyQt5`，使用独立的 `.quest_stream_ui.json` 配置。终端界面首次启动或 Reset Defaults 后为 WiFi / 8080，按所选连接方式重新设置。终端界面的 `1` / `2` / `3` / `4` 分别打开 Controls、Left Hand、Right Hand、Logs，`Tab` 切换页面。
