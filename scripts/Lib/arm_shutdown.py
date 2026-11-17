"""动作脚本统一收尾：保持原控制模式，先回真实零位，再停止电机。"""

from contextvars import ContextVar
from functools import wraps
import signal
import time

import numpy as np
from scipy.interpolate import BPoly, PPoly


_active_arms = ContextVar("openarm_active_motion_arms", default=None)
_active_signals = ContextVar("openarm_shutdown_signals", default=None)
DEFAULT_KP = [30., 50., 60., 25., 15., 10., 10.]
DEFAULT_KD = [3., 5., 6., 2.5, 1.5, 1., 1.]


class ShutdownSignals:
    """首次中断进入回零；再次中断直接结束回零，仍发送停止命令。"""

    def __init__(self):
        self.previous = {}
        self.requested = False
        self.immediate = False
        self.homing = False
        self.stopping = False

    def handle(self, signum, _frame):
        if self.stopping:
            self.immediate = True
            return
        if self.requested or self.homing:
            self.immediate = True
            print("\n再次中断：立即停止。", flush=True)
        else:
            self.requested = True
            print("\n收到退出请求：先回真实零位。", flush=True)
        raise KeyboardInterrupt

    def install(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            self.previous[sig] = signal.signal(sig, self.handle)
        return self

    def restore(self):
        for sig, previous in self.previous.items():
            signal.signal(sig, previous)
        self.previous.clear()


def register_arm(robot, label):
    """在创建成功后立即登记，后续初始化失败也会进入统一收尾。"""
    arms = _active_arms.get()
    if arms is not None and not any(item["robot"] is robot for item in arms):
        arms.append({"label": label, "robot": robot})
    return robot


def forget_registered_arms():
    """用于在处理离线绘图前已经完成回零和停止的脚本。"""
    arms = _active_arms.get()
    if arms is not None:
        arms.clear()


def _states(arms):
    result, seen = [], set()
    for item in arms:
        state = dict(item) if isinstance(item, dict) else {"label": item[0], "robot": item[1]}
        if id(state["robot"]) not in seen:
            result.append(state)
            seen.add(id(state["robot"]))
    return result


def _vector(value, count, label):
    value = np.asarray(value, dtype=float)
    if value.shape != (count,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{label}维度或数值无效")
    return value


def _feedback(robot):
    count = robot.motor_count
    position = _vector(robot.get_current_pos(), count, "关节位置反馈")
    velocity = _vector(robot.get_current_vel(), count, "关节速度反馈")
    # SDK 的 position 可能是缓存值；有电机原始反馈时一并检查时间和故障。
    for motor in getattr(robot, "Motors", [])[:count]:
        state = motor.get_current_motor_state()
        if hasattr(state, "fault") and int(state.fault) != 0:
            raise RuntimeError(f"电机 ID{motor.get_motor_id()} 故障")
        if hasattr(state, "time"):
            age = time.time() - float(state.time)
            if not np.isfinite(age) or not -.05 <= age <= .5:
                raise RuntimeError(f"电机 ID{motor.get_motor_id()} 反馈过期")
    return position, velocity



def _curve_bounds(curve, derivative=0):
    """五次回零曲线的解析极值，覆盖帧与帧之间的峰值。"""
    coefficients = curve.derivative(derivative).c[:, 0, :]
    span = curve.x[-1] - curve.x[0]
    bounds = []
    for values in coefficients.T:
        roots = np.roots(np.polyder(values))
        points = [0., span] + [float(r.real) for r in roots
                              if abs(r.imag) < 1e-9 and 0. < r.real < span]
        extrema = np.polyval(values, points)
        bounds.append((np.min(extrema), np.max(extrema)))
    return np.asarray(bounds).T


def _home_curve(state, min_duration, speed):
    """从实测位置和速度接续，终点位置、速度及加速度均为零。"""
    robot = state["robot"]
    q, v = state["start"], state["start_velocity"]
    count = robot.motor_count
    configured_v = _vector(getattr(robot, "velocity_limits", np.full(count, speed)), count, "速度限制")
    configured_a = _vector(getattr(robot, "acceleration_limits", np.full(count, 2.)), count, "加速度限制")
    if np.any(configured_v <= 0.) or np.any(configured_a <= 0.):
        raise ValueError("回零速度、加速度限制必须大于零")
    # 已有速度须先平滑消减，不能在第一帧直接跳成回零低速。
    allowed_v = np.maximum(np.minimum(configured_v, speed), abs(v))
    base = max(float(min_duration), 1.875 * float(np.max(abs(q))) / speed)
    candidates = sorted({max(float(min_duration), base * factor)
                         for factor in [*(.8**i for i in range(10)), *(1.25**i for i in range(14))]})
    zeros = np.zeros(count)
    for duration in candidates:
        curve = PPoly.from_bernstein_basis(BPoly.from_derivatives(
            [0., duration], [[q, v, zeros], [zeros, zeros, zeros]],
        ))
        lower, upper = _curve_bounds(curve)
        vl, vh = _curve_bounds(curve, 1)
        al, ah = _curve_bounds(curve, 2)
        if (np.all(lower >= state["lower"] - 1e-9)
                and np.all(upper <= state["upper"] + 1e-9)
                and np.all(np.maximum(abs(vl), abs(vh)) <= allowed_v + 1e-9)
                and np.all(np.maximum(abs(al), abs(ah)) <= configured_a + 1e-9)):
            return curve, duration
    raise RuntimeError("当前位置和速度无法衔接为限位内的平滑回零曲线")


def home_and_stop(arms, *, control_mode="posvel", kp=None, kd=None,
                  controller=None, min_duration=3., speed=.35, tolerance=.05,
                  settle_timeout=8., signals=None):
    """逐臂验证反馈，协调平滑回零，最后独立停止每条臂。

    MIT、位置速度、速度、软件力矩控制均保持调用方模式。单臂反馈失效
    不阻止另一臂收尾；缺失反馈、第二次中断时只停止，不能保证自动回零。
    """
    if control_mode not in ("posvel", "mit", "velocity", "torque"):
        raise ValueError(f"未知回零模式: {control_mode}")
    states = _states(arms)
    if not states:
        return True
    signals = signals or _active_signals.get()
    if signals is not None:
        signals.homing = True
    active, errors = [], []
    interrupted = False
    try:
        if signals is not None and signals.immediate:
            return False
        for state in states:
            robot, label = state["robot"], state["label"]
            try:
                robot.send_get_motor_state_cmd()
                robot.motor_send_cmd()
                time.sleep(.03)
                position, velocity = _feedback(robot)
                count = robot.motor_count
                lower = _vector(robot.joint_limits["lower"], count, "限位下界")
                upper = _vector(robot.joint_limits["upper"], count, "限位上界")
                if np.any(lower > 0.) or np.any(upper < 0.):
                    raise ValueError("当前关节配置不包含真实零位")
                if np.any(position < lower - .05) or np.any(position > upper + .05):
                    raise ValueError("当前位置越界，不能生成回零路径")
                # 在模型边界附近的反馈舍入误差不应导致起点命令被 SDK 拒绝。
                state["start"] = np.clip(position, lower, upper)
                state["start_velocity"] = velocity.copy()
                state["lower"], state["upper"] = lower, upper
                state["kp"] = _vector(state.get("kp", kp if kp is not None else DEFAULT_KP), count, "回零 KP")
                state["kd"] = _vector(state.get("kd", kd if kd is not None else DEFAULT_KD), count, "回零 KD")
                state["torque_limit"] = _vector(robot.max_torque, count, "最大力矩")
                state["last_error"] = float(np.max(abs(position)))
                if np.max(abs(position)) <= tolerance and np.max(abs(velocity)) <= .15:
                    print(f"{label}当前已在真实零位，最大误差 {state['last_error']:.3f} rad", flush=True)
                else:
                    state["curve"], state["duration"] = _home_curve(state, min_duration, speed)
                    state["velocity_limit"] = np.maximum(speed, abs(velocity))
                    active.append(state)
            except Exception as error:
                errors.append(f"{label}: {error}")
                print(f"\n{label}无法回零: {error}", flush=True)
                try:
                    robot.set_stop()
                    state["stopped"] = True
                except Exception as stop_error:
                    errors.append(f"{label}停止失败: {stop_error}")
        if not active:
            return not errors
        start = time.monotonic()
        next_deadline = time.perf_counter()
        print("\n退出收尾：平滑回到真实零位（全部关节为 0）...", flush=True)
        while active:
            elapsed = time.monotonic() - start
            for state in list(active):
                robot, label = state["robot"], state["label"]
                try:
                    actual, actual_velocity = _feedback(robot)
                    duration = state["duration"]
                    curve_time = min(elapsed, duration)
                    target = state["curve"](curve_time)
                    velocity = state["curve"](curve_time, 1)
                    if elapsed >= duration:
                        target = np.zeros(robot.motor_count)
                        velocity = np.zeros(robot.motor_count)
                    if control_mode == "posvel":
                        # 此接口的速度为幅值；目标方向由位置决定。
                        success = robot.Joint_Pos_Vel(target, np.maximum(abs(velocity), .03), robot.max_torque, iswait=False)
                    elif control_mode == "velocity":
                        command = velocity + 2. * (target - actual)
                        command = np.clip(command, -state["velocity_limit"], state["velocity_limit"])
                        success = robot.Joint_Vel(command)
                    else:
                        gravity = _vector(robot.get_Gravity(actual), robot.motor_count, "重力前馈")
                        if control_mode == "torque":
                            torque = state["kp"] * (target - actual) + state["kd"] * (velocity - actual_velocity) + gravity
                            zeros = np.zeros(robot.motor_count)
                            success = robot.pos_vel_tqe_kp_kd(zeros, zeros, np.clip(torque, -state["torque_limit"], state["torque_limit"]), zeros, zeros)
                        elif controller is not None:
                            success = controller(state, target, velocity, actual, actual_velocity)
                        else:
                            success = robot.pos_vel_tqe_kp_kd(target, velocity, np.clip(gravity, -state["torque_limit"], state["torque_limit"]), state["kp"], state["kd"])
                    if success is False:
                        raise RuntimeError("回零命令被拒绝")
                    state["last_error"] = float(np.max(abs(actual)))
                    # 到达插值末端后继续闭环确认零位，绝不按时间直接宣告到位。
                    if elapsed >= duration and state["last_error"] <= tolerance and np.max(abs(actual_velocity)) <= .15:
                        if control_mode == "velocity":
                            robot.Joint_Vel(np.zeros(robot.motor_count))
                        print(f"{label}已回零，最大误差 {state['last_error']:.3f} rad", flush=True)
                        active = [item for item in active if item is not state]
                    elif elapsed > duration + settle_timeout:
                        raise RuntimeError(f"回零未到位，最大误差 {state['last_error']:.3f} rad")
                except Exception as error:
                    errors.append(f"{label}: {error}")
                    print(f"\n{label}回零中止: {error}", flush=True)
                    active = [item for item in active if item is not state]
                    try:
                        robot.set_stop()
                        state["stopped"] = True
                    except Exception as stop_error:
                        errors.append(f"{label}停止失败: {stop_error}")
            if active:
                next_deadline += .01
                delay = next_deadline - time.perf_counter()
                if delay > 0.:
                    time.sleep(delay)
                else:
                    next_deadline = time.perf_counter()
    except KeyboardInterrupt:
        interrupted = True
        print("\n回零中断，正在停止电机。", flush=True)
    finally:
        if signals is not None:
            signals.stopping = True
        for state in states:
            if state.get("stopped"):
                continue
            try:
                state["robot"].set_stop()
            except Exception as error:
                errors.append(f"{state['label']}停止失败: {error}")
                print(f"\n停止{state['label']}失败: {error}", flush=True)
        if signals is not None:
            signals.homing = False
            signals.stopping = False
    return not errors and not interrupted


def return_home_on_exit(*, control_mode="posvel", kp=None, kd=None):
    """覆盖整个 main，包括启动定位中断、异常和正常结束的回零收尾。"""
    def decorate(function):
        @wraps(function)
        def run(*args, **kwargs):
            arms = []
            arm_token = _active_arms.set(arms)
            signals = ShutdownSignals().install()
            signal_token = _active_signals.set(signals)
            try:
                return function(*args, **kwargs)
            except KeyboardInterrupt:
                print("\n程序被中断，进入回零收尾。", flush=True)
            finally:
                try:
                    home_and_stop(arms, control_mode=control_mode, kp=kp, kd=kd, signals=signals)
                finally:
                    signals.restore()
                    _active_signals.reset(signal_token)
                    _active_arms.reset(arm_token)
        return run
    return decorate
