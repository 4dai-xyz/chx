#!/usr/bin/env python3
"""
Go2 MuJoCo 运动控制教学 demo。  # 本文件用于第 1 步剩余 5 个运动 demo 的稳定可视化复现。

重要说明：  # 这段说明非常重要，避免把教学动画误当成实机控制程序。
1. 本文件直接读取 MuJoCo 模型，并用“运动学位姿更新 + 腿部步态动画”的方式展示运动效果。  # 这样可以稳定看到直行、转向、绕圈和路径点跟踪。
2. 本文件不发布 rt/lowcmd，也不连接真实 Go2。  # 所以它不会误伤实机，也不会和正在跑的 DDS 仿真桥抢控制权。
3. 本文件适合初学阶段理解“高层速度指令 -> 机器人位姿变化 -> 路径点状态机”。  # 后续再把速度指令接到真正的低层控制器或宇树 SportClient。
4. 如果要上真实 Go2，必须改用宇树高层运动接口 SportClient.Move()/StopMove() 并保留急停。  # 不要把本文件的运动学动画直接移植到实机。
"""

from __future__ import annotations

import argparse  # 解析命令行参数，例如 --demo straight。
import math  # 提供 sin/cos/atan2 等基础数学函数。
import time  # 控制仿真刷新节奏，让画面接近真实时间。
from dataclasses import dataclass  # 用 dataclass 表达简单配置结构，代码更清楚。
from pathlib import Path  # 用 Path 拼接工程路径，比手写字符串更稳。
from typing import Callable, Sequence, Tuple  # 类型标注，方便你以后读代码和改代码。

import mujoco  # MuJoCo 物理/模型接口，用来加载 Go2 MJCF 模型。
import mujoco.viewer  # MuJoCo 官方 viewer，用来实时显示三维画面。
import numpy as np  # 用 numpy 管理关节角数组和插值计算。


PROJECT_ROOT = Path(__file__).resolve().parents[2]  # /home/ros/unitree_dev，保证从任何目录运行都能找到模型。
GO2_SCENE_XML = PROJECT_ROOT / "src/unitree_mujoco/unitree_robots/go2/scene.xml"  # Go2 平地场景文件。


STAND_UP_JOINT_POS = np.array(  # 官方 stand_go2.py 中的 Go2 站立姿态，顺序是 FR, FL, RR, RL，每条腿 3 个关节。
    [
        0.00571868,
        0.608813,
        -1.21763,
        -0.00571868,
        0.608813,
        -1.21763,
        0.00571868,
        0.608813,
        -1.21763,
        -0.00571868,
        0.608813,
        -1.21763,
    ],
    dtype=float,
)


STAND_DOWN_JOINT_POS = np.array(  # 官方 stand_go2.py 中的 Go2 趴下姿态，用于进入/退出 demo 时平滑过渡。
    [
        0.0473455,
        1.22187,
        -2.44375,
        -0.0473455,
        1.22187,
        -2.44375,
        0.0473455,
        1.22187,
        -2.44375,
        -0.0473455,
        1.22187,
        -2.44375,
    ],
    dtype=float,
)


LEG_NAMES = ("FR", "FL", "RR", "RL")  # 4 条腿的名字，FR=右前，FL=左前，RR=右后，RL=左后。
RIGHT_LEG_INDEX = {0, 2}  # FR 和 RR 是右侧腿，转向动画会用到左右侧区别。
DIAGONAL_PHASE = (0.0, math.pi, math.pi, 0.0)  # 小跑步态常见的对角腿相位：FR/RL 同相，FL/RR 同相。


@dataclass
class Pose2D:
    """机器人在平面上的位姿。"""  # x/y/yaw 是后续导航和路径点跟踪的核心状态。

    x: float = 0.0  # 世界坐标系 x，单位 m。
    y: float = 0.0  # 世界坐标系 y，单位 m。
    yaw: float = 0.0  # 世界坐标系偏航角，单位 rad。


@dataclass(frozen=True)
class MotionLimits:
    """高层运动限幅。"""  # 先把速度压低，保证初学 demo 的画面稳定且容易观察。

    max_vx: float = 0.16  # 最大前进速度，单位 m/s；教学 demo 不需要太快。
    max_yaw_rate: float = 0.45  # 最大原地转向角速度，单位 rad/s。
    max_acc_vx: float = 0.25  # 最大线速度变化率，单位 m/s^2；用于平滑启动和停止。
    max_acc_yaw_rate: float = 0.80  # 最大角速度变化率，单位 rad/s^2；用于平滑转向。


def clamp(value: float, low: float, high: float) -> float:
    """把数值限制到 [low, high] 区间。"""  # 所有速度指令都先限幅，避免突然跳变。

    return max(low, min(high, value))  # Python 的 max/min 组合就是最简单的限幅写法。


def wrap_to_pi(angle: float) -> float:
    """把角度规整到 [-pi, pi]。"""  # 路径点跟踪里计算航向误差时必须做这一步。

    return (angle + math.pi) % (2.0 * math.pi) - math.pi  # 这是移动机器人里很常见的角度归一化公式。


def yaw_to_quat(yaw: float) -> np.ndarray:
    """把平面 yaw 角转换成 MuJoCo free joint 使用的四元数 [w, x, y, z]。"""  # Go2 根节点姿态用四元数存储。

    half_yaw = 0.5 * yaw  # 四元数公式使用半角。
    return np.array([math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)], dtype=float)  # 只绕 z 轴旋转。


def lerp_array(start: np.ndarray, end: np.ndarray, phase: float) -> np.ndarray:
    """数组线性插值。"""  # 用来从趴下姿态平滑过渡到站立姿态。

    safe_phase = clamp(phase, 0.0, 1.0)  # 防止相位超过 0 到 1。
    return start * (1.0 - safe_phase) + end * safe_phase  # 标准线性插值公式。


def smooth_step(phase: float) -> float:
    """平滑插值曲线。"""  # 比普通线性插值更柔和，开始和结束时速度更小。

    safe_phase = clamp(phase, 0.0, 1.0)  # 先保证输入范围正确。
    return safe_phase * safe_phase * (3.0 - 2.0 * safe_phase)  # smoothstep 曲线，常用于动画过渡。


class RateLimiter:
    """速度变化率限制器。"""  # 让速度慢慢升上去、慢慢降下来，模拟真实机器人的安全限幅。

    def __init__(self, limits: MotionLimits) -> None:
        self.limits = limits  # 保存线速度和角速度的限幅参数。
        self.vx = 0.0  # 当前实际输出线速度，初始为 0。
        self.yaw_rate = 0.0  # 当前实际输出角速度，初始为 0。

    def update(self, target_vx: float, target_yaw_rate: float, dt: float) -> Tuple[float, float]:
        """根据目标速度和 dt 更新平滑后的速度。"""  # 这一步相当于一个非常简单的速度控制器。

        safe_target_vx = clamp(target_vx, -self.limits.max_vx, self.limits.max_vx)  # 目标线速度限幅。
        safe_target_yaw_rate = clamp(target_yaw_rate, -self.limits.max_yaw_rate, self.limits.max_yaw_rate)  # 目标角速度限幅。

        max_delta_vx = self.limits.max_acc_vx * dt  # 本次循环线速度最多允许变化多少。
        max_delta_yaw = self.limits.max_acc_yaw_rate * dt  # 本次循环角速度最多允许变化多少。

        self.vx += clamp(safe_target_vx - self.vx, -max_delta_vx, max_delta_vx)  # 逐步逼近目标线速度。
        self.yaw_rate += clamp(safe_target_yaw_rate - self.yaw_rate, -max_delta_yaw, max_delta_yaw)  # 逐步逼近目标角速度。

        return self.vx, self.yaw_rate  # 返回平滑后的速度，用它更新机器人位姿。


class Go2KinematicMujocoDemo:
    """Go2 MuJoCo 教学可视化控制器。"""  # 这个类封装模型加载、姿态更新、步态动画和 viewer 刷新。

    def __init__(self, scene_xml: Path = GO2_SCENE_XML, dt: float = 0.01) -> None:
        if not scene_xml.exists():  # 先检查模型文件是否存在，避免报错信息太隐晦。
            raise FileNotFoundError(f"找不到 Go2 场景文件: {scene_xml}")  # 明确告诉你缺的是哪个文件。

        self.model = mujoco.MjModel.from_xml_path(str(scene_xml))  # 从 MJCF XML 加载 MuJoCo 模型。
        self.data = mujoco.MjData(self.model)  # 创建 MuJoCo 数据对象，保存 qpos/qvel/sensor 等运行时状态。
        self.dt = dt  # 教学动画的控制周期，默认 0.01s，也就是 100Hz。
        self.pose = Pose2D()  # 当前机器人平面位姿。
        self.base_height = 0.34  # Go2 站立时根节点高度，过低会像趴着，过高会像悬空。
        self.walk_phase = 0.0  # 腿部步态相位，持续累加后驱动 sin/cos 动画。
        self.rate_limiter = RateLimiter(MotionLimits())  # 创建速度平滑器。
        self.viewer = None  # viewer 稍后在 run 里启动，便于控制生命周期。
        self.reset_pose()  # 初始化 MuJoCo qpos，让机器人以趴下姿态出现在原点。

    def reset_pose(self) -> None:
        """重置机器人到原点附近。"""  # 每个 demo 开始前都调用，保证实验条件一致。

        self.pose = Pose2D()  # 平面位姿归零。
        self.walk_phase = 0.0  # 步态相位归零。
        self.rate_limiter = RateLimiter(MotionLimits())  # 速度平滑器归零。
        self.data.qpos[:] = 0.0  # 清空所有广义坐标，避免继承上一次 demo 的状态。
        self.data.qpos[0:3] = np.array([self.pose.x, self.pose.y, 0.18], dtype=float)  # 初始高度较低，对应趴下。
        self.data.qpos[3:7] = yaw_to_quat(self.pose.yaw)  # 根节点朝向设为 0。
        self.data.qpos[7:19] = STAND_DOWN_JOINT_POS  # 12 个腿部关节设置为趴下姿态。
        mujoco.mj_forward(self.model, self.data)  # 根据 qpos 更新几何体位置，刷新 viewer 前必须调用。

    def apply_robot_state(self, joint_pos: np.ndarray, height: float | None = None) -> None:
        """把当前平面位姿和关节角写入 MuJoCo 数据。"""  # 这是运动学可视化 demo 的核心输出。

        root_height = self.base_height if height is None else height  # 如果没有指定高度，就使用默认站立高度。
        self.data.qpos[0] = self.pose.x  # 写入世界坐标 x。
        self.data.qpos[1] = self.pose.y  # 写入世界坐标 y。
        self.data.qpos[2] = root_height  # 写入根节点高度。
        self.data.qpos[3:7] = yaw_to_quat(self.pose.yaw)  # 写入根节点 yaw 对应的四元数。
        self.data.qpos[7:19] = joint_pos  # 写入 12 个腿部关节角。
        self.data.qvel[:] = 0.0  # 教学动画不依赖物理积分，速度清零可以避免残留抖动。
        mujoco.mj_forward(self.model, self.data)  # 通知 MuJoCo 根据新的 qpos 重新计算场景。

    def sync_viewer(self) -> bool:
        """刷新 viewer 并返回窗口是否仍在运行。"""  # 如果用户关闭窗口，主循环就会退出。

        if self.viewer is None:  # 理论上 run 之后才会有 viewer，这里做保护。
            return False  # 没有 viewer 就认为不应继续运行。

        self.viewer.cam.lookat[:] = np.array([self.pose.x, self.pose.y, 0.25], dtype=float)  # 相机始终看向机器狗附近。
        self.viewer.sync()  # 刷新 MuJoCo 窗口画面。
        return self.viewer.is_running()  # 返回窗口状态，供 demo 循环判断。

    def stand_up(self, duration: float = 1.8) -> None:
        """从趴下平滑站起来。"""  # 每个运动 demo 开始前先站稳，视觉上也更清楚。

        start_time = time.perf_counter()  # 记录过渡开始时间。
        while self.viewer is not None and self.viewer.is_running():  # 只要窗口还在就继续过渡。
            elapsed = time.perf_counter() - start_time  # 计算已经过了多久。
            phase = smooth_step(elapsed / duration)  # 把时间映射到平滑的 0 到 1 相位。
            joint_pos = lerp_array(STAND_DOWN_JOINT_POS, STAND_UP_JOINT_POS, phase)  # 关节从趴下插值到站立。
            height = 0.18 * (1.0 - phase) + self.base_height * phase  # 根节点高度也同步抬高。
            self.apply_robot_state(joint_pos, height=height)  # 把插值结果写进 MuJoCo。
            if not self.sync_viewer():  # 如果窗口被关闭，就停止过渡。
                return  # 直接返回，避免访问已关闭 viewer。
            if elapsed >= duration:  # 到达指定过渡时间。
                break  # 站立完成。
            time.sleep(self.dt)  # 控制刷新频率，避免 CPU 占满。

    def stand_down(self, duration: float = 1.2) -> None:
        """从站立平滑趴下。"""  # demo 结束时回到安全姿态，形成完整闭环。

        start_time = time.perf_counter()  # 记录过渡开始时间。
        while self.viewer is not None and self.viewer.is_running():  # 只要窗口还开着就继续。
            elapsed = time.perf_counter() - start_time  # 已经过的时间。
            phase = smooth_step(elapsed / duration)  # 平滑相位。
            joint_pos = lerp_array(STAND_UP_JOINT_POS, STAND_DOWN_JOINT_POS, phase)  # 关节从站立插值到趴下。
            height = self.base_height * (1.0 - phase) + 0.18 * phase  # 根节点高度逐渐降低。
            self.apply_robot_state(joint_pos, height=height)  # 写入 MuJoCo 状态。
            if not self.sync_viewer():  # 如果窗口关闭。
                return  # 立即退出。
            if elapsed >= duration:  # 到达过渡时间。
                break  # 趴下完成。
            time.sleep(self.dt)  # 保持稳定刷新。

    def make_walk_joint_pose(self, vx: float, yaw_rate: float) -> np.ndarray:
        """根据高层速度指令生成腿部步态动画。"""  # 这里只做可视化步态，不是可迁移到实机的控制策略。

        speed_level = clamp(abs(vx) / 0.16 + abs(yaw_rate) / 0.45, 0.0, 1.0)  # 用速度大小决定腿摆动幅度。
        if speed_level < 0.03:  # 如果速度接近 0。
            return STAND_UP_JOINT_POS.copy()  # 直接保持标准站立，不额外摆腿。

        gait_frequency = 1.8 + 0.6 * speed_level  # 速度越大，腿摆动频率略高。
        self.walk_phase += 2.0 * math.pi * gait_frequency * self.dt  # 按时间推进步态相位。

        joint_pos = STAND_UP_JOINT_POS.copy()  # 从标准站立姿态开始叠加小幅摆动。
        forward_sign = 1.0 if vx >= 0.0 else -1.0  # 前进/后退决定腿部前后摆动方向。
        turn_sign = 1.0 if yaw_rate >= 0.0 else -1.0  # 左转/右转决定左右腿动画差异。

        for leg_index, leg_name in enumerate(LEG_NAMES):  # 逐条腿生成 3 个关节目标角。
            base = leg_index * 3  # 当前腿在 12 维关节数组中的起始下标。
            phase = self.walk_phase + DIAGONAL_PHASE[leg_index]  # 加上对角步态相位。
            right_side = leg_index in RIGHT_LEG_INDEX  # 判断当前腿是不是右腿。
            side_sign = 1.0 if right_side else -1.0  # 右腿为 +1，左腿为 -1。

            forward_amp = 0.10 * clamp(abs(vx) / 0.16, 0.0, 1.0)  # 直行动画的腿前后摆动幅度。
            turn_amp = 0.08 * clamp(abs(yaw_rate) / 0.45, 0.0, 1.0)  # 转向动画的左右腿差速幅度。
            lift_amp = 0.16 * speed_level  # 抬腿幅度，速度越大视觉上越明显。
            hip_amp = 0.025 * speed_level  # 髋外展小幅摆动，让步态看起来更自然。

            side_turn_direction = side_sign * turn_sign  # 左右腿在原地转向时的运动方向相反。
            thigh_wave = math.cos(phase)  # 大腿前后摆动波形。
            lift_wave = max(0.0, math.sin(phase))  # 只在摆动相抬腿，落地相不抬腿。

            joint_pos[base + 0] += side_sign * hip_amp * math.sin(phase)  # 髋关节小幅左右摆动，增加视觉上的重心转移。
            joint_pos[base + 1] += forward_sign * forward_amp * thigh_wave  # 直行时大腿前后摆动。
            joint_pos[base + 1] += side_turn_direction * turn_amp * thigh_wave  # 转向时左右腿形成差速摆动。
            joint_pos[base + 2] -= lift_amp * lift_wave  # 小腿在摆动相折起，形成抬脚效果。

            _ = leg_name  # 保留 leg_name 变量，方便你调试时打印腿名；当前不打印以免刷屏。

        return joint_pos  # 返回最终 12 维关节角。

    def step_velocity(self, target_vx: float, target_yaw_rate: float) -> Tuple[float, float]:
        """执行一步高层速度控制。"""  # 这相当于 Move(vx, 0, yaw_rate) 的教学版。

        vx, yaw_rate = self.rate_limiter.update(target_vx, target_yaw_rate, self.dt)  # 先做速度限幅和平滑。
        self.pose.yaw = wrap_to_pi(self.pose.yaw + yaw_rate * self.dt)  # 根据角速度更新 yaw。
        self.pose.x += math.cos(self.pose.yaw) * vx * self.dt  # 根据当前朝向把局部前进速度投影到世界 x。
        self.pose.y += math.sin(self.pose.yaw) * vx * self.dt  # 根据当前朝向把局部前进速度投影到世界 y。
        joint_pos = self.make_walk_joint_pose(vx, yaw_rate)  # 生成与当前速度匹配的腿部动画。
        self.apply_robot_state(joint_pos)  # 写入 MuJoCo 并刷新模型状态。
        return vx, yaw_rate  # 返回实际执行速度，便于打印和调试。

    def run_for(
        self,
        duration: float,
        command_fn: Callable[[float], Tuple[float, float]],
        status_fn: Callable[[float, float, float], str] | None = None,
    ) -> None:
        """按给定速度函数运行一段时间。"""  # straight/turn/circle 这类固定时长 demo 都用它。

        start_time = time.perf_counter()  # demo 起始时间。
        next_print_time = 0.0  # 控制终端打印频率，避免刷屏。
        while self.viewer is not None and self.viewer.is_running():  # 主循环，窗口关闭就停止。
            elapsed = time.perf_counter() - start_time  # 已运行时间。
            if elapsed >= duration:  # 如果到达目标时长。
                break  # 退出当前 demo。

            target_vx, target_yaw_rate = command_fn(elapsed)  # 根据当前时间计算目标速度。
            vx, yaw_rate = self.step_velocity(target_vx, target_yaw_rate)  # 执行一步速度控制。

            if status_fn is not None and elapsed >= next_print_time:  # 如果需要打印状态，并且到了打印时刻。
                print(status_fn(elapsed, vx, yaw_rate), flush=True)  # 打印当前 demo 状态。
                next_print_time = elapsed + 0.5  # 每 0.5 秒打印一次。

            if not self.sync_viewer():  # 刷新窗口，如果窗口关闭。
                break  # 退出循环。
            time.sleep(self.dt)  # 保持接近实时速度。

    def stop_smoothly(self, duration: float = 1.0) -> None:
        """平滑停下来并保持站立。"""  # 所有 demo 结束时都先把速度归零。

        self.run_for(  # 复用 run_for 做 0 速度过渡。
            duration,
            command_fn=lambda _elapsed: (0.0, 0.0),  # 目标线速度和角速度都为 0。
            status_fn=None,  # 停止阶段不打印状态。
        )
        self.apply_robot_state(STAND_UP_JOINT_POS.copy())  # 最后把腿放回标准站立姿态。
        self.sync_viewer()  # 刷新一次画面。

    def demo_straight(self) -> None:
        """Demo 2：直行。"""  # 让 Go2 低速向前走几秒。

        print("\n[Demo 2] 直行：以约 0.12 m/s 向前走 5 秒。", flush=True)  # 说明当前 demo。
        self.run_for(  # 固定速度运行。
            duration=5.0,
            command_fn=lambda _elapsed: (0.12, 0.0),  # vx=0.12m/s，yaw_rate=0。
            status_fn=lambda elapsed, vx, yaw_rate: (
                f"t={elapsed:4.1f}s  vx={vx:+.2f} m/s  yaw_rate={yaw_rate:+.2f} rad/s  "
                f"pose=({self.pose.x:+.2f}, {self.pose.y:+.2f}, yaw={self.pose.yaw:+.2f})"
            ),  # 打印时间、速度和位姿。
        )
        self.stop_smoothly()  # 平滑停止。

    def demo_turn(self) -> None:
        """Demo 3：原地转向。"""  # 让 Go2 在原地低速左转。

        print("\n[Demo 3] 原地转向：以约 0.35 rad/s 左转 5 秒。", flush=True)  # 说明当前 demo。
        self.run_for(
            duration=5.0,
            command_fn=lambda _elapsed: (0.0, 0.35),  # vx=0，yaw_rate=0.35rad/s。
            status_fn=lambda elapsed, vx, yaw_rate: (
                f"t={elapsed:4.1f}s  vx={vx:+.2f} m/s  yaw_rate={yaw_rate:+.2f} rad/s  "
                f"yaw={self.pose.yaw:+.2f} rad"
            ),  # 打印 yaw 变化。
        )
        self.stop_smoothly()  # 平滑停止。

    def demo_distance_stop(self, target_distance: float = 1.0) -> None:
        """Demo 4：直行一段距离后停止。"""  # 用里程累计判断何时停车。

        print(f"\n[Demo 4] 直行定距停止：向前走 {target_distance:.2f} m 后停止。", flush=True)  # 说明当前 demo。
        start_x = self.pose.x  # 记录起点 x。
        start_y = self.pose.y  # 记录起点 y。
        next_print_time = 0.0  # 控制打印频率。

        while self.viewer is not None and self.viewer.is_running():  # 主循环。
            distance = math.hypot(self.pose.x - start_x, self.pose.y - start_y)  # 计算从起点到当前位置的直线距离。
            if distance >= target_distance:  # 到达目标距离。
                print(f"已到达 {distance:.2f} m，开始停止。", flush=True)  # 打印停止原因。
                break  # 退出运动循环。

            remaining = target_distance - distance  # 剩余距离。
            target_vx = clamp(0.35 * remaining, 0.04, 0.12)  # 越接近目标速度越小，但最低保持一点速度。
            vx, yaw_rate = self.step_velocity(target_vx, 0.0)  # 执行直行速度。

            now = time.perf_counter()  # 当前时间。
            if now >= next_print_time:  # 如果到了打印时刻。
                print(
                    f"distance={distance:.2f}/{target_distance:.2f} m  "
                    f"vx={vx:+.2f} m/s  pose=({self.pose.x:+.2f}, {self.pose.y:+.2f})",
                    flush=True,
                )  # 打印距离进度。
                next_print_time = now + 0.5  # 半秒后再打印。

            if not self.sync_viewer():  # 刷新窗口。
                break  # 窗口关闭就退出。
            time.sleep(self.dt)  # 稳定刷新周期。

        self.stop_smoothly()  # 到点后平滑停车。

    def demo_circle(self) -> None:
        """Demo 5：低速绕圈。"""  # 同时给 vx 和 yaw_rate，机器人就会沿弧线走。

        vx_cmd = 0.10  # 线速度，单位 m/s。
        yaw_cmd = 0.25  # 角速度，单位 rad/s。
        radius = vx_cmd / yaw_cmd  # 圆半径约等于 vx / yaw_rate。
        print(f"\n[Demo 5] 低速绕圈：vx={vx_cmd:.2f} m/s, yaw_rate={yaw_cmd:.2f} rad/s, 半径约 {radius:.2f} m。", flush=True)
        self.run_for(
            duration=18.0,
            command_fn=lambda _elapsed: (vx_cmd, yaw_cmd),  # 同时前进和转向。
            status_fn=lambda elapsed, vx, yaw_rate: (
                f"t={elapsed:4.1f}s  vx={vx:+.2f} m/s  yaw_rate={yaw_rate:+.2f} rad/s  "
                f"pose=({self.pose.x:+.2f}, {self.pose.y:+.2f}, yaw={self.pose.yaw:+.2f})"
            ),  # 打印绕圈过程中的位姿。
        )
        self.stop_smoothly()  # 平滑停止。

    def demo_waypoints(self, waypoints: Sequence[Tuple[float, float]] | None = None) -> None:
        """Demo 6：3 个路径点跟踪。"""  # 这是从运动控制走向导航的第一步。

        if waypoints is None:  # 如果调用者没有传入路径点。
            waypoints = ((0.8, 0.0), (0.8, 0.5), (0.2, 0.5))  # 默认 3 个点，形成一个折线路径。

        print("\n[Demo 6] 路径点跟踪：按顺序走向 3 个目标点。", flush=True)  # 说明当前 demo。
        for index, waypoint in enumerate(waypoints, start=1):  # 打印所有目标点。
            print(f"  目标点 {index}: x={waypoint[0]:+.2f}, y={waypoint[1]:+.2f}", flush=True)  # 方便你对照画面。

        waypoint_index = 0  # 当前正在跟踪的路径点下标。
        next_print_time = 0.0  # 控制打印频率。

        while self.viewer is not None and self.viewer.is_running():  # 主循环。
            if waypoint_index >= len(waypoints):  # 如果所有目标点都已经完成。
                print("所有路径点已完成，开始平滑停车。", flush=True)  # 打印完成信息。
                break  # 退出循环。

            target_x, target_y = waypoints[waypoint_index]  # 当前目标点。
            dx = target_x - self.pose.x  # x 方向误差。
            dy = target_y - self.pose.y  # y 方向误差。
            distance = math.hypot(dx, dy)  # 到目标点的距离。
            target_yaw = math.atan2(dy, dx)  # 指向目标点的期望朝向。
            yaw_error = wrap_to_pi(target_yaw - self.pose.yaw)  # 当前朝向和目标朝向的误差。

            if distance < 0.08:  # 到达阈值，8cm 内认为到达。
                print(f"到达目标点 {waypoint_index + 1}: ({target_x:+.2f}, {target_y:+.2f})", flush=True)  # 打印到达信息。
                waypoint_index += 1  # 切换到下一个目标点。
                self.stop_smoothly(duration=0.4)  # 到点后短暂停稳，便于观察状态切换。
                continue  # 继续下一轮循环。

            if abs(yaw_error) > 0.20:  # 朝向误差较大时，先转向再前进。
                target_vx = 0.0  # 大角度误差时不前进，避免走偏。
                target_yaw_rate = clamp(1.3 * yaw_error, -0.35, 0.35)  # P 控制器：角速度与角度误差成比例。
                state = "先转向"  # 当前状态文本。
            else:  # 朝向基本对准后再前进。
                target_vx = clamp(0.45 * distance, 0.04, 0.12)  # 距离越远速度越大，接近目标时自动减速。
                target_yaw_rate = clamp(1.0 * yaw_error, -0.22, 0.22)  # 前进时保留小角速度修正航向。
                state = "向目标前进"  # 当前状态文本。

            vx, yaw_rate = self.step_velocity(target_vx, target_yaw_rate)  # 执行本轮速度指令。

            now = time.perf_counter()  # 当前时间。
            if now >= next_print_time:  # 如果到了打印时刻。
                print(
                    f"目标 {waypoint_index + 1}/{len(waypoints)}  {state}  "
                    f"dist={distance:.2f} m  yaw_err={yaw_error:+.2f} rad  "
                    f"vx={vx:+.2f}  yaw_rate={yaw_rate:+.2f}  "
                    f"pose=({self.pose.x:+.2f}, {self.pose.y:+.2f}, {self.pose.yaw:+.2f})",
                    flush=True,
                )  # 打印导航状态。
                next_print_time = now + 0.5  # 半秒打印一次。

            if not self.sync_viewer():  # 刷新窗口。
                break  # 窗口关闭则退出。
            time.sleep(self.dt)  # 保持实时刷新。

        self.stop_smoothly()  # 路径完成后平滑停止。

    def run_selected_demo(self, demo_name: str) -> None:
        """运行指定 demo。"""  # 这里把命令行 --demo 和具体函数对应起来。

        demo_map = {  # 字符串到函数的映射，后续加新 demo 只需要在这里加一行。
            "straight": self.demo_straight,
            "turn": self.demo_turn,
            "distance": self.demo_distance_stop,
            "circle": self.demo_circle,
            "waypoints": self.demo_waypoints,
        }

        if demo_name == "all":  # all 表示依次运行 2 到 6。
            for name in ("straight", "turn", "distance", "circle", "waypoints"):  # 按学习顺序逐个运行。
                self.reset_pose()  # 每个 demo 前重置，方便观察单个效果。
                self.stand_up()  # 先站起来。
                demo_map[name]()  # 运行当前 demo。
                time.sleep(0.6)  # demo 之间停顿一下。
            return  # 所有 demo 完成。

        if demo_name not in demo_map:  # 如果用户输入了不存在的 demo 名。
            raise ValueError(f"未知 demo: {demo_name}")  # 抛出明确错误。

        self.reset_pose()  # 单个 demo 开始前重置。
        self.stand_up()  # 先站立。
        demo_map[demo_name]()  # 执行指定 demo。
        self.stand_down()  # 单个 demo 结束后趴下，保持闭环流程。

    def run(self, demo_name: str) -> None:
        """启动 viewer 并运行 demo。"""  # 外部 main 函数只需要调用这个入口。

        print("启动 MuJoCo viewer。关闭窗口或 Ctrl+C 可以结束程序。", flush=True)  # 给用户一个终端提示。
        viewer = mujoco.viewer.launch_passive(self.model, self.data)  # 启动 MuJoCo 被动 viewer，写法和 unitree_mujoco.py 保持一致。
        self.viewer = viewer  # 保存 viewer 引用，供类内其他方法刷新画面。
        try:  # 用 try/finally 包住 viewer，保证程序退出时能释放窗口资源。
            self.sync_viewer()  # 先刷新一次初始画面。
            self.run_selected_demo(demo_name)  # 运行用户选择的 demo。
            self.stop_smoothly(duration=0.6)  # 最后确保速度归零。
            time.sleep(0.5)  # 留半秒让你看到最终姿态。
        finally:  # 无论正常结束还是 Ctrl+C，都尝试关闭 viewer。
            if hasattr(viewer, "close"):  # 不同 MuJoCo 版本的 viewer 句柄略有差异，所以先判断 close 是否存在。
                viewer.close()  # 关闭 MuJoCo viewer，释放窗口。


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""  # 运行时通过 --demo 选择不同复现任务。

    parser = argparse.ArgumentParser(description="Go2 MuJoCo 运动控制教学 demo")  # 创建参数解析器。
    parser.add_argument(
        "--demo",
        choices=("straight", "turn", "distance", "circle", "waypoints", "all"),
        default="straight",
        help="选择要运行的 demo：straight 直行，turn 原地转向，distance 定距停止，circle 绕圈，waypoints 路径点，all 全部运行。",
    )  # demo 选择参数。
    parser.add_argument(
        "--dt",
        type=float,
        default=0.01,
        help="控制周期，默认 0.01 秒；数值越小越流畅，但 CPU 占用越高。",
    )  # 控制循环周期。
    return parser.parse_args()  # 返回解析后的参数对象。


def main() -> None:
    """程序主入口。"""  # Python 脚本运行时从这里开始。

    args = parse_args()  # 读取命令行参数。
    demo = Go2KinematicMujocoDemo(dt=args.dt)  # 创建 Go2 MuJoCo demo 控制器。
    try:  # 用 try 包起来，保证 Ctrl+C 时能优雅退出。
        demo.run(args.demo)  # 运行用户选择的 demo。
    except KeyboardInterrupt:  # 用户按 Ctrl+C。
        print("\n用户中断，程序退出。", flush=True)  # 打印退出提示。


if __name__ == "__main__":
    main()  # 标准 Python 入口写法。
