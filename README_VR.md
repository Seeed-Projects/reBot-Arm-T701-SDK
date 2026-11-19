# T701-HT VR Teleoperation

[简体中文](README_VR_CN.md)

Use the left and right VR controllers to control the corresponding arms, including end-effector pose tracking and gripper control. This demo has been tested on Quest. Pico users can connect through the same WebXR → WebSocket → UDP pipeline using a WebXR-capable headset browser.

## 1. Prepare the environment

Complete the [SDK setup and serial mapping](README.md), check feedback, and verify zero positions. Run the commands below from the SDK root. Activate the environment in each new terminal:

```bash
conda activate t701-ht
```

## 2. Start streaming

```bash
python Quest_controller_stream/quest_stream_tui.py
```

Use Up/Down to select a field, Left/Right to adjust it, `E` to edit addresses or ports, and Enter to activate a button. Select **Stop Server** before changing modes, then set the values for your connection:

| Setting | USB (Wired) | WiFi (Wireless) |
| --- | --- | --- |
| Mode | USB | WiFi |
| Force HTTPS | False | True |
| HTTP Host / Port | 0.0.0.0 / 8080 | 0.0.0.0 / 8080 |
| WS Host / Port | 0.0.0.0 / 8765 | 0.0.0.0 / 8765 |
| UDP IP / Port | 127.0.0.1 / 5005 | 127.0.0.1 / 5005 |
| Verify Port | 5005 | 5005 |

### USB (Wired)

Enable developer mode on the headset, connect it to the computer with a USB data cable, and authorize USB debugging. On Ubuntu, install a missing ADB with `sudo apt install adb`, then check the connection in another terminal:

```bash
adb devices
```

The device must show status `device`. If it shows `unauthorized`, confirm authorization in the headset.

Select **Start Server** to configure `adb reverse` for ports 8080 and 8765. Open this URL in the headset browser:

```text
http://localhost:8080/index.html?ws=ws://localhost:8765
```

Set the WebSocket input to `ws://localhost:8765`. USB uses HTTP / WS and does not need an HTTPS certificate.

### WiFi (Wireless)

Connect the headset and computer to the same LAN. No USB cable or ADB is needed. The computer needs OpenSSL; on Ubuntu, install it with `sudo apt install openssl` if necessary. Find the computer's actual LAN IP, not a Docker or VPN adapter address:

```bash
hostname -I
```

Keep the certificate path `.certs/quest-controller.crt` and private key path `.certs/quest-controller.key`, then select **Start Server**. WiFi uses HTTPS / WSS and generates a self-signed certificate when the certificate files are missing.

Open the UI's WiFi URL in the headset browser. For example, if the computer's IP is `192.168.1.100`:

```text
https://192.168.1.100:8080/index.html?ws=wss://192.168.1.100:8765
```

Replace the example with the computer's actual IP. On first access, handle the browser's certificate prompt and reload. The WebSocket input must be `wss://ACTUAL_COMPUTER_IP:8765`. The listening address `0.0.0.0` is not an access address. When streaming and control run on the same computer, the UDP destination remains `127.0.0.1:5005`.

### Check the connection

Check that the page's WebSocket input matches the selected mode rather than settings saved for another connection. Select **Start XR Stream**. Confirm WebSocket is `Connected`, XR is `Running`, and both controllers show `Tracking`.

Select **Start Verify** in the computer UI. Check that packet count and controller data change, then select **Stop Verify**. Keep the server, the selected connection, and headset XR stream running.

## 3. Start dual-arm control

**Stop Verify and any other UDP 5005 receiver first. After receiving data, the controller moves the left arm and then the right arm to their VR initial poses. Clear the motion range and return path before starting.**

In another terminal, from the SDK root:

```bash
conda activate t701-ht
cd scripts
python t701_ht_vr_dual_cartesian_qp_control.py
```

This entry controls both arms directly. The left arm uses CAN1 and the right arm CAN2, with configurations in `T701-HT_robot_param/T701-HT_left.yaml` and `T701-HT_right.yaml`.

- Hold **Squeeze** to control the corresponding arm; release it to hold position.
- **Trigger** controls the corresponding gripper while Squeeze is held.
- The primary button, labeled A in the program, smoothly resets that arm to its initial joint angles for this run.
- **Ctrl+C** returns the arms to zero in MIT mode and stops the motors. Wait for shutdown to finish. Press **Q** in the stream UI to stop the server and quit.

An occupied port or no packets within 10 seconds causes the controller to exit before initializing hardware. Check that Verify is stopped, XR streaming is active, and the UDP destination is `127.0.0.1:5005`.

## Other entries

Without the terminal UI, choose one command from the SDK root. Do not run it alongside another stream server:

```bash
# USB: ADB forwarding, HTTP / WS
python Quest_controller_stream/start_stream_server.py --http-port 8080 --ws-port 8765 --udp-ip 127.0.0.1 --udp-port 5005 --adb auto

# WiFi: no ADB, HTTPS / WSS automatically enabled
python Quest_controller_stream/start_stream_server.py --http-port 8080 --ws-port 8765 --udp-ip 127.0.0.1 --udp-port 5005 --adb never
```

The desktop UI, `Quest_controller_stream/quest_stream_ui.py`, requires the optional `PyQt5` dependency and uses a separate `.quest_stream_ui.json` configuration. The terminal UI defaults and Reset Defaults use WiFi / 8080; configure your selected connection mode. In the terminal UI, `1` / `2` / `3` / `4` open Controls, Left Hand, Right Hand, and Logs. Use Tab to switch pages.
