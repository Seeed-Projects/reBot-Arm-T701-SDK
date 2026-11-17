# T701-HT 与 Pinocchio（匹诺曹）

## 1. 实机准备

按 [T701-HT SDK 中文说明](README_CN.md) 安装依赖并检查串口映射。

### 安全提示

- 启动前确认机械臂固定牢靠、关节零点正确，并取下负载。首次试跑选单臂。
- 选臂后，动作示例可能立即移动。启动前清空整段活动范围，包括头顶和回零路径。全程现场看护，手和身体避开关节及夹爪夹点。
- 重力补偿启动和退出前先支撑好机械臂，避免补偿力矩撤除后下坠。
- 带自动回零的示例首次按 `Ctrl+C` 会触发回零。回零期间再按一次可中断回零并发送停止命令。
- 键盘碰撞预测在到达 VR 初始姿态后启用，仅依据 URDF 网格。启动和回零时也要预留空间，并为夹爪和实际制动距离留出余量。

从项目根目录进入示例目录。

```bash
cd scripts
```

运行例程后，按提示输入 `1`、`2` 或 `3` 选择左臂、右臂或双臂。

## 2. 运动学与重力补偿

| 示例 | 运行命令 | 预期行为 |
| --- | --- | --- |
| 实时正运动学（FK） | `python t701_ht_forward_kinematics_test.py` | 读取关节角，约每秒打印末端位置（米）和 RPY（度）。 |
| 当前位姿 FK → IK 验证 | `python t701_ht_inverse_kinematics_test.py` | 读取关节角，用当前位姿做逆解，打印末端位置和最大关节角误差。 |
| 重力补偿 | `python t701_ht_gravity_compensation.py` | 施加限幅后的 `0.8 × G(q)`，减轻手动拖动负担，打印补偿力矩。 |
| 重力补偿并输出 FK | `python t701_ht_gravity_compensation_with_fk.py` | 施加限幅后的 `G(q)`，打印关节角、补偿力矩和末端位姿。 |
| 关节阻抗与重力前馈 | `python t701_ht_joint_impedance_with_gravity.py` | 主动移向并保持预设关节姿态，叠加重力补偿，打印总力矩。 |
| 关节阻抗、重力和摩擦前馈 | `python t701_ht_joint_impedance_with_gravity_friction.py` | 同样移向并保持预设姿态，额外施加摩擦补偿，打印总力矩。 |

FK 和 IK 验证只查询反馈，不下发运动目标。按 `Ctrl+C` 退出时，两种验证和两种重力补偿直接停止电机，两个关节阻抗示例先回零再停止。

## 3. 笛卡尔控制

| 示例 | 运行命令 | 预期行为 |
| --- | --- | --- |
| IK 到点、关节运动与回零 | `python t701_ht_inv_posvel_control.py` | 先回关节零位。目标位姿逆解通过误差检查后，用 `moveJ` 到点，再回零停止。 |
| QP 举臂、头顶画圆与回零 | `python t701_ht_qp_ik.py` | 先到 VR 初始姿态，再举臂、在头顶画圆，最后回零停止。 |
| MoveL 直线、对角线和正方形 | `python t701_ht_moveL_pos_control.py` | 保持末端目标朝向，连续走直线、对角线和正方形路径。 |
| MoveL 绕轴旋转和圆锥运动 | `python t701_ht_moveL_rotate_control.py` | 保持末端目标位置，连续绕局部轴旋转并做圆锥运动。 |

两个 MoveL 示例先到 VR 初始姿态，连续演示 5 秒，再回零停止。初始定位和回零另计。

## 4. 键盘控制

| 示例 | 运行命令 | 预期行为 |
| --- | --- | --- |
| 末端位姿 | `python t701_ht_keyboard_cartesian_pos_control.py` | 按键逐步改变末端目标位姿，逆解成功时移动，无解时维持上一关节目标。 |
| 末端速度 | `python t701_ht_keyboard_cartesian_vel_control.py` | 按住键时末端持续平移或旋转，松开后速度指令逐步降至零。 |
| 末端速度与网格碰撞预测 | `python t701_ht_keyboard_cartesian_vel_collision_control.py` | 按键控制末端速度，预测到碰撞或关节越限时下发零速度并提示。 |
| 笛卡尔阻抗 | `python t701_ht_keyboard_cartesian_impedance_control.py` | 按键调整末端目标位姿，通过关节力矩和重力补偿柔顺跟踪。 |
| 零空间姿态锁定 | `python t701_ht_keyboard_cartesian_pd_nullspace_lock_control.py` | 跟踪按键指定的末端位姿，同时尽量维持参考关节姿态。 |

在本地或 SSH 终端运行。选臂后会自动移动到 VR 初始姿态。W/S、A/D、Q/E 沿 X、Y、Z 移动，1/2、3/4、5/6 绕对应轴旋转。零空间脚本用 R 重新锁定参考姿态。按 `Esc` 或 `Ctrl+C` 退出时会回零并停止。

碰撞例程使用 `T701-HT_URDF/meshes/` 的 STL，预测未来 0.20 秒的碰撞和关节越限。双臂模式也检查两臂之间的碰撞。
