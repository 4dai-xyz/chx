#!/usr/bin/env python3
"""
Go2 MuJoCo 低速稳定步态实验台。

这个文件不是“最终可实机迁移的行走控制器”，而是当前学习阶段的第二步：
  先让 Go2 在 MuJoCo 中稳定站住；
  再用非常小的键盘速度指令触发低幅度步态；
  观察 cmd_vel、步态相位、目标关节角、PD 力矩、姿态稳定性之间的关系。

为什么要重写得这么保守：
  之前的键盘一次按键就是 vx=0.5、yaw=1.0，步频 2Hz，大腿摆幅 0.35rad。
  这些参数对“还没有足端轨迹规划/质心控制/MPC/RL 策略”的手写步态来说太激进。
  现在先把幅度降下来，目标是“不倒、可解释、可调参”，然后再一点点加速度。

推荐运行：
  cd /home/ros/unitree_dev
  env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    .venv-unitree/bin/python projects/go2_control_demos/go2_walk_runner.py

无窗口稳定性自检：
  env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    .venv-unitree/bin/python projects/go2_control_demos/go2_walk_runner.py --headless-test --test-vx 0.15

按键：
  ↑/↓    前进/后退，默认每次只增加 0.05
  ←/→    小幅横向实验，默认很小，不追求明显横移
  Q/E    小幅转向实验，默认很小，不追求快速原地转
  空格/R 急停，速度命令清零
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


# ============================================================
# 1. 路径与姿态常量
# ============================================================
PROJECT_ROOT = Path(__file__).resolve().parents[2]  # 工程根目录：/home/ros/unitree_dev。
GO2_SCENE_XML = PROJECT_ROOT / "src/unitree_mujoco/unitree_robots/go2/scene.xml"  # Go2 MuJoCo 场景。


# unitree_mujoco 官方 stand_go2.py 的稳定站立角，顺序是 actuator 顺序：FR, FL, RR, RL。
STAND_UP_ACTUATOR = np.array([
    0.00571868, 0.608813, -1.21763,    # FR: hip, thigh, calf。
    -0.00571868, 0.608813, -1.21763,   # FL: hip, thigh, calf。
    0.00571868, 0.608813, -1.21763,    # RR: hip, thigh, calf。
    -0.00571868, 0.608813, -1.21763,   # RL: hip, thigh, calf。
], dtype=np.float64)


# 对角小跑相位，顺序是 actuator 腿顺序：FR, FL, RR, RL。
PHASE_OFFSET = np.array([0.0, math.pi, math.pi, 0.0], dtype=np.float64)  # FR+RL 同相，FL+RR 同相。


@dataclass
class WalkConfig:
    """低速步态实验参数，集中放在这里方便你之后逐项调参。"""

    scene_xml: Path = GO2_SCENE_XML  # MuJoCo 场景文件。
    kp: float = 50.0  # PD 比例增益，沿用官方 stand_go2.py 的稳定值。
    kd: float = 3.5  # PD 阻尼增益，沿用官方 stand_go2.py 的稳定值。
    stand_time: float = 2.0  # 从 keyframe home 平滑站起的时间。
    duration: float = 240.0  # viewer 模式最长运行时间。
    headless_duration: float = 8.0  # 无窗口自检时长。
    base_freq: float = 0.8  # 基础步频，先用低频，避免腿部抽搐。
    max_thigh_amp: float = 0.10  # 大腿最大摆幅，之前 0.35 太大，先降到 0.10。
    min_thigh_amp: float = 0.05  # 非零速度下最小摆幅，太小则几乎不动。
    min_forward_bias: float = 0.18  # 非零前进命令下的最小大腿前向偏置，用于克服原地踏步后滑。
    max_forward_bias: float = 0.26  # 最大大腿前向偏置，太大会压低机身、增加失稳风险。
    calf_amp: float = 0.04  # 小腿折叠幅度，先用很小的抬腿动作。
    hip_amp: float = 0.015  # hip 侧摆幅度，只用于非常小的横移/稳定实验。
    cmd_step: float = 0.1  # 每次按键改变的速度命令，之前 0.5 太大。
    max_vx: float = 0.25  # 前进/后退命令最大值，先限制在低速。
    max_vy: float = 0.3  # 横移命令最大值，当前只是实验，不追求明显横移。
    max_yaw: float = 0.4  # 转向命令最大值，当前只是实验，不追求快速原地转。
    cmd_rate: float = 0.35  # 命令缓变速度，每秒最多变化多少，防止突然冲击。
    gait_ramp_time: float = 1.0  # 从站立切换到步态时，步态幅度缓慢爬升。
    pitch_k: float = 0.25  # 小幅俯仰校正，不做强控制。
    roll_k: float = 0.20  # 小幅横滚校正，不做强控制。


# ============================================================
# 2. MuJoCo 工具函数
# ============================================================
def build_joint_mappings(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    """
    建立 actuator 顺序和 qpos 顺序之间的映射。

    qpos 顺序由 MuJoCo body 树决定，Go2 当前是 FL, FR, RL, RR。
    actuator 顺序由 go2.xml 的 <actuator> 决定，当前是 FR, FL, RR, RL。
    """
    actuator_to_qpos = np.zeros(model.nu, dtype=np.int32)  # actuator[i] 对应 qpos[7 + j] 的 j。
    qpos_to_actuator = np.zeros(model.nu, dtype=np.int32)  # qpos[7 + j] 对应 actuator[i] 的 i。

    for actuator_id in range(model.nu):  # 遍历 12 个电机。
        joint_id = int(model.actuator_trnid[actuator_id][0])  # 查询 actuator 驱动的 joint。
        qpos_index = int(model.jnt_qposadr[joint_id] - 7)  # 转成相对于 qpos[7:] 的关节索引。
        actuator_to_qpos[actuator_id] = qpos_index  # 保存 actuator -> qpos。
        qpos_to_actuator[qpos_index] = actuator_id  # 保存 qpos -> actuator。

    return actuator_to_qpos, qpos_to_actuator


def actuator_joint_ranges(model: mujoco.MjModel) -> np.ndarray:
    """读取每个 actuator 对应关节的角度范围，用于目标关节角限幅。"""
    ranges = np.zeros((model.nu, 2), dtype=np.float64)  # 每行是 [lo, hi]。
    for actuator_id in range(model.nu):  # 遍历 actuator。
        joint_id = int(model.actuator_trnid[actuator_id][0])  # 找到 actuator 对应 joint。
        ranges[actuator_id] = model.jnt_range[joint_id]  # 保存 joint range。
    return ranges


def clip_actuator_targets(target_actuator: np.ndarray, joint_ranges: np.ndarray) -> np.ndarray:
    """限制目标关节角，避免手写步态把关节推到极限。"""
    clipped = target_actuator.copy()  # 不直接改输入数组，便于调试。
    for i in range(clipped.size):  # 每个关节单独限幅。
        lo, hi = joint_ranges[i]  # 读取该关节范围。
        margin = min(0.05, max(0.0, (hi - lo) * 0.03))  # 留一点余量，避免贴着机械限位。
        clipped[i] = np.clip(clipped[i], lo + margin, hi - margin)  # 应用限幅。
    return clipped


def clip_ctrl(model: mujoco.MjModel, ctrl: np.ndarray) -> np.ndarray:
    """按 MuJoCo actuator ctrlrange 限制力矩，避免 PD 输出过大。"""
    clipped = ctrl.copy()  # 不直接改输入数组。
    for i in range(model.nu):  # 每个电机单独限幅。
        lo, hi = model.actuator_ctrlrange[i]  # 读取力矩范围。
        clipped[i] = np.clip(clipped[i], lo, hi)  # 限制力矩。
    return clipped


def body_pitch_roll(data: mujoco.MjData) -> tuple[float, float]:
    """从 floating base 四元数提取 pitch 和 roll，用于姿态监控。"""
    w, x, y, z = data.qpos[3:7]  # MuJoCo free joint 四元数顺序是 [w, x, y, z]。
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))  # 绕 y 轴俯仰角。
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))  # 绕 x 轴横滚角。
    return float(pitch), float(roll)


def has_fallen(data: mujoco.MjData) -> bool:
    """简单跌倒保护：机身太低或姿态角太大就停止实验。"""
    pitch, roll = body_pitch_roll(data)  # 获取姿态角。
    return bool(data.qpos[2] < 0.16 or abs(pitch) > 0.80 or abs(roll) > 0.80)  # 返回是否不安全。


def smoothstep(x: float) -> float:
    """0 到 1 的平滑插值函数，比线性插值更柔和。"""
    x = min(1.0, max(0.0, x))  # 先限制到 [0, 1]。
    return x * x * (3.0 - 2.0 * x)  # smoothstep 曲线。


# ============================================================
# 3. 命令输入和缓变
# ============================================================
class CommandState:
    """保存键盘期望命令和实际平滑命令。"""

    def __init__(self, cfg: WalkConfig):
        self.cfg = cfg  # 保存参数。
        self.desired = np.zeros(3, dtype=np.float32)  # 键盘直接修改的期望命令。
        self.actual = np.zeros(3, dtype=np.float32)  # 控制器真正使用的平滑命令。

    def stop(self) -> None:
        """急停：期望命令和实际命令都清零。"""
        self.desired[:] = 0.0  # 清空期望命令。
        self.actual[:] = 0.0  # 清空实际命令。
        print("\r[cmd] stop vx=+0.00 vy=+0.00 yaw=+0.00        ", end="", flush=True)

    def add(self, dvx: float, dvy: float, dyaw: float) -> None:
        """键盘增量修改期望命令，并做低速限幅。"""
        self.desired[0] = np.clip(self.desired[0] + dvx, -self.cfg.max_vx, self.cfg.max_vx)  # 限制 vx。
        self.desired[1] = np.clip(self.desired[1] + dvy, -self.cfg.max_vy, self.cfg.max_vy)  # 限制 vy。
        self.desired[2] = np.clip(self.desired[2] + dyaw, -self.cfg.max_yaw, self.cfg.max_yaw)  # 限制 yaw。
        print(
            f"\r[desired] vx={self.desired[0]:+.2f} vy={self.desired[1]:+.2f} "
            f"yaw={self.desired[2]:+.2f}        ",
            end="",
            flush=True,
        )

    def update(self, dt: float) -> np.ndarray:
        """把实际命令按最大变化率缓慢追近期望命令。"""
        max_delta = self.cfg.cmd_rate * dt  # 本步每个维度最多变化多少。
        delta = np.clip(self.desired - self.actual, -max_delta, max_delta)  # 限制命令变化率。
        self.actual += delta.astype(np.float32)  # 更新实际命令。
        return self.actual.copy()  # 返回实际命令副本。


def make_key_callback(cmd_state: CommandState):
    """创建 MuJoCo viewer 键盘回调。"""
    step = cmd_state.cfg.cmd_step  # 每次按键的命令增量。

    def callback(keycode: int) -> None:
        from mujoco.glfw import glfw  # 延迟导入，避免 headless 模式初始化 GLFW。

        if keycode == glfw.KEY_UP:
            cmd_state.add(+step, 0.0, 0.0)  # ↑：增加低速前进命令。
        elif keycode == glfw.KEY_DOWN:
            cmd_state.add(-step, 0.0, 0.0)  # ↓：增加低速后退命令。
        elif keycode == glfw.KEY_LEFT:
            cmd_state.add(0.0, +step, 0.0)  # ←：小幅左移实验。
        elif keycode == glfw.KEY_RIGHT:
            cmd_state.add(0.0, -step, 0.0)  # →：小幅右移实验。
        elif keycode == glfw.KEY_Q:
            cmd_state.add(0.0, 0.0, +step)  # Q：小幅左转实验。
        elif keycode == glfw.KEY_E:
            cmd_state.add(0.0, 0.0, -step)  # E：小幅右转实验。
        elif keycode in (glfw.KEY_SPACE, glfw.KEY_R):
            cmd_state.stop()  # 空格/R：急停。

    return callback


# ============================================================
# 4. 保守步态生成器
# ============================================================
class ConservativeTrot:
    """低速保守对角步态生成器。"""

    def __init__(self, cfg: WalkConfig):
        self.cfg = cfg  # 保存参数。
        self.motion_start_time: float | None = None  # 记录从站立切到运动的时刻，用于幅度缓慢爬升。

    def compute(self, sim_time: float, cmd_vel: np.ndarray) -> np.ndarray:
        """
        根据低速 cmd_vel 生成 12 个目标关节角，输出 actuator 顺序。

        这不是足端轨迹规划，只是教学用的保守正弦步态。
        先让你看懂“速度命令如何影响关节目标”，后续再切到 RL/MPC。
        """
        vx, vy, yaw = float(cmd_vel[0]), float(cmd_vel[1]), float(cmd_vel[2])  # 拆出三个速度命令。
        speed = math.sqrt(vx * vx + vy * vy + yaw * yaw)  # 计算命令大小。

        if speed < 1e-3:
            self.motion_start_time = None  # 回到站立时重置步态爬升计时。
            return STAND_UP_ACTUATOR.copy()  # 零命令就保持稳定站立姿态。

        if self.motion_start_time is None:
            self.motion_start_time = sim_time  # 第一次收到非零命令时记录开始时间。

        ramp = smoothstep((sim_time - self.motion_start_time) / self.cfg.gait_ramp_time)  # 步态幅度缓慢爬升。
        freq = self.cfg.base_freq + 0.4 * min(abs(vx), self.cfg.max_vx)  # 前进越大，频率稍微增加。
        vx_ratio = min(1.0, abs(vx) / max(self.cfg.max_vx, 1e-6))  # vx 归一化到 [0, 1]。
        thigh_amp = self.cfg.min_thigh_amp + (self.cfg.max_thigh_amp - self.cfg.min_thigh_amp) * vx_ratio  # 大腿摆幅。
        forward_bias_mag = self.cfg.min_forward_bias + (
            self.cfg.max_forward_bias - self.cfg.min_forward_bias
        ) * vx_ratio  # 大腿前向偏置幅度；没有这个偏置时，正弦踏步会在当前模型里缓慢后滑。
        forward_bias = math.copysign(forward_bias_mag, vx)  # vx>0 前进偏置为正，vx<0 后退偏置为负。
        direction = -1.0 if vx >= 0.0 else 1.0  # 摆腿相位符号；配合 forward_bias 做前后运动。
        yaw_ratio = np.clip(yaw / max(self.cfg.max_yaw, 1e-6), -1.0, 1.0)  # 转向命令归一化。
        vy_ratio = np.clip(vy / max(self.cfg.max_vy, 1e-6), -1.0, 1.0) if self.cfg.max_vy > 0 else 0.0  # 横移命令归一化。

        target = STAND_UP_ACTUATOR.copy()  # 从稳定站立姿态开始叠加小幅步态。

        for leg in range(4):  # leg: 0=FR, 1=FL, 2=RR, 3=RL。
            phase = 2.0 * math.pi * freq * (sim_time - (self.motion_start_time or sim_time)) + PHASE_OFFSET[leg]  # 当前腿相位。
            sin_phase = math.sin(phase)  # 正弦相位，用于大腿前后摆。
            cos_phase = math.cos(phase)  # 余弦相位，用于 hip 小幅侧摆。
            is_right = leg in (0, 2)  # 右腿是 FR/RR。
            is_front = leg in (0, 1)  # 前腿是 FR/FL。

            hip = leg * 3 + 0  # 当前腿 hip actuator 索引。
            thigh = leg * 3 + 1  # 当前腿 thigh actuator 索引。
            calf = leg * 3 + 2  # 当前腿 calf actuator 索引。

            # 前进/后退：主要通过 thigh 小幅前后摆动实现。
            turn_scale = 1.0 + (0.25 * yaw_ratio if is_right else -0.25 * yaw_ratio)  # 转向时左右腿步幅略不对称。
            target[thigh] += ramp * (
                forward_bias + direction * thigh_amp * turn_scale * sin_phase
            )  # 叠加大腿目标角；forward_bias 负责确定前后推进方向。

            # 小腿：摆动相稍微折叠，支撑相稍微伸展，幅度非常小，先求稳定。
            target[calf] += ramp * (
                -self.cfg.calf_amp * max(0.0, sin_phase)
                + 0.4 * self.cfg.calf_amp * max(0.0, -sin_phase)
            )  # 叠加小腿目标角。

            # hip：只做很小的横向/转向辅助，避免左右摇晃失控。
            side_sign = 1.0 if is_right else -1.0  # 右腿和左腿方向相反。
            front_sign = 1.0 if is_front else -1.0  # 前腿和后腿方向相反。
            target[hip] += ramp * self.cfg.hip_amp * side_sign * vy_ratio * cos_phase  # 小幅横移实验。
            target[hip] += ramp * 0.5 * self.cfg.hip_amp * front_sign * yaw_ratio  # 小幅转向实验。

        return target  # 返回 actuator 顺序目标角。


def apply_posture_feedback(target: np.ndarray, data: mujoco.MjData, cfg: WalkConfig) -> np.ndarray:
    """给目标关节角叠加很小的姿态反馈，主要用于抑制慢速倾斜。"""
    corrected = target.copy()  # 不直接修改输入。
    pitch, roll = body_pitch_roll(data)  # 获取当前俯仰和横滚。

    pitch_correction = np.clip(-cfg.pitch_k * pitch, -0.06, 0.06)  # 限制俯仰修正量。
    roll_correction = np.clip(cfg.roll_k * roll, -0.04, 0.04)  # 限制横滚修正量。

    for thigh in (1, 4, 7, 10):  # 四条腿 thigh 索引。
        corrected[thigh] += pitch_correction  # 前后统一修正，避免强烈改变步态。

    for leg, hip in enumerate((0, 3, 6, 9)):  # 四条腿 hip 索引。
        is_right = leg in (0, 2)  # 右腿标记。
        corrected[hip] += (1.0 if is_right else -1.0) * roll_correction  # 左右腿反向修正横滚。

    return corrected  # 返回修正后的目标角。


# ============================================================
# 5. Runner
# ============================================================
class LowSpeedWalkRunner:
    """低速步态实验 runner，负责站立、步态、PD、保护和可视化。"""

    def __init__(self, cfg: WalkConfig):
        self.cfg = cfg  # 保存配置。
        if not cfg.scene_xml.exists():
            raise FileNotFoundError(f"找不到场景文件: {cfg.scene_xml}")

        self.model = mujoco.MjModel.from_xml_path(str(cfg.scene_xml))  # 加载 MuJoCo 模型。
        self.data = mujoco.MjData(self.model)  # 创建仿真数据。
        self.dt = float(self.model.opt.timestep)  # MuJoCo 步长。
        self.actuator_to_qpos, self.qpos_to_actuator = build_joint_mappings(self.model)  # 顺序映射。
        self.joint_ranges = actuator_joint_ranges(self.model)  # actuator 顺序下的关节范围。
        self.gait = ConservativeTrot(cfg)  # 保守步态生成器。
        self.target_qpos = np.zeros(self.model.nu, dtype=np.float64)  # 当前 qpos 顺序目标角。
        self.step_count = 0  # 步计数。

    def reset(self) -> None:
        """从 MuJoCo keyframe home 初始化，避免自己手填 base 高度导致穿地或悬空。"""
        if self.model.nkey > 0:
            self.data.qpos[:] = self.model.key_qpos[0].copy()  # 使用 go2.xml 的 home keyframe。
        else:
            self.data.qpos[:7] = [0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0]  # 没有 keyframe 时兜底。
            self.data.qpos[7:] = STAND_UP_ACTUATOR[self.actuator_to_qpos]  # 关节设为站立。
        self.data.qvel[:] = 0.0  # 初速度清零。
        self.data.ctrl[:] = 0.0  # 力矩清零。
        self.target_qpos = self.data.qpos[7:].copy()  # 起始目标等于当前角。
        self.step_count = 0  # 步计数清零。

    def standup_target(self, sim_time: float) -> None:
        """站立阶段目标：从 keyframe home 平滑过渡到官方稳定站姿。"""
        start_qpos = self.model.key_qpos[0, 7:].copy() if self.model.nkey > 0 else self.data.qpos[7:].copy()  # 起点。
        stand_qpos = STAND_UP_ACTUATOR[self.actuator_to_qpos]  # 终点。
        s = smoothstep(sim_time / self.cfg.stand_time)  # 平滑进度。
        self.target_qpos = (1.0 - s) * start_qpos + s * stand_qpos  # 插值得到目标角。

    def walk_target(self, sim_time: float, cmd_vel: np.ndarray) -> None:
        """运动阶段目标：保守步态 + 小幅姿态反馈 + 关节限幅。"""
        target_actuator = self.gait.compute(sim_time, cmd_vel)  # 生成 actuator 顺序目标角。
        target_actuator = apply_posture_feedback(target_actuator, self.data, self.cfg)  # 加小幅姿态反馈。
        target_actuator = clip_actuator_targets(target_actuator, self.joint_ranges)  # 关节角限幅。
        self.target_qpos = target_actuator[self.actuator_to_qpos]  # 转为 qpos 顺序。

    def pd_step(self) -> None:
        """执行一步 PD 控制并推进物理。"""
        tau_qpos = (
            self.cfg.kp * (self.target_qpos - self.data.qpos[7:19])
            + self.cfg.kd * (0.0 - self.data.qvel[6:18])
        )  # qpos 顺序力矩。
        ctrl = tau_qpos[self.qpos_to_actuator]  # 转 actuator 顺序。
        self.data.ctrl[:] = clip_ctrl(self.model, ctrl)  # 力矩限幅后写入 MuJoCo。
        mujoco.mj_step(self.model, self.data)  # 推进物理。
        self.step_count += 1  # 步计数加一。

    def run_step(self, sim_time: float, cmd_vel: np.ndarray) -> None:
        """执行一轮控制：站立阶段或步态阶段。"""
        if sim_time < self.cfg.stand_time:
            self.standup_target(sim_time)  # 前 stand_time 秒只站起来。
        else:
            self.walk_target(sim_time, cmd_vel)  # 之后根据命令生成步态。
        self.pd_step()  # 每个物理步都执行 PD。

    def print_status(self, sim_time: float, cmd_vel: np.ndarray, prefix: str = "[状态]") -> None:
        """打印当前状态，方便判断是否稳定。"""
        pitch, roll = body_pitch_roll(self.data)  # 姿态角。
        print(
            f"\r{prefix} t={sim_time:5.2f}s h={self.data.qpos[2]:.3f} "
            f"pitch={pitch:+.2f} roll={roll:+.2f} "
            f"cmd=({cmd_vel[0]:+.2f},{cmd_vel[1]:+.2f},{cmd_vel[2]:+.2f})",
            end="",
            flush=True,
        )

    def run_headless(self, test_cmd: np.ndarray) -> bool:
        """无窗口自检，返回 True 表示没有触发跌倒保护。"""
        self.reset()  # 初始化。
        sim_time = 0.0  # 仿真时间。
        total_steps = int(self.cfg.headless_duration / self.dt)  # 总步数。
        min_height = 999.0  # 记录最低高度。
        max_pitch = 0.0  # 记录最大俯仰。
        max_roll = 0.0  # 记录最大横滚。
        start_x = float(self.data.qpos[0])  # 记录起点 x。

        for _ in range(total_steps):
            cmd = np.zeros(3, dtype=np.float32) if sim_time < self.cfg.stand_time else test_cmd  # 站稳后再给测试命令。
            self.run_step(sim_time, cmd)  # 执行控制。
            pitch, roll = body_pitch_roll(self.data)  # 获取姿态。
            min_height = min(min_height, float(self.data.qpos[2]))  # 更新最低高度。
            max_pitch = max(max_pitch, abs(pitch))  # 更新最大俯仰。
            max_roll = max(max_roll, abs(roll))  # 更新最大横滚。
            sim_time += self.dt  # 更新时间。
            if sim_time > self.cfg.stand_time + 0.5 and has_fallen(self.data):
                print(
                    f"\n[失败] t={sim_time:.2f}s h={self.data.qpos[2]:.3f} "
                    f"min_h={min_height:.3f} max_pitch={max_pitch:.2f} max_roll={max_roll:.2f}"
                )
                return False

        dx = float(self.data.qpos[0] - start_x)  # 计算 x 位移，手写步态可能方向与视角不同。
        print(
            f"\n[通过] t={sim_time:.2f}s h={self.data.qpos[2]:.3f} dx={dx:+.3f} "
            f"min_h={min_height:.3f} max_pitch={max_pitch:.2f} max_roll={max_roll:.2f}"
        )
        return True

    def run_viewer(self) -> None:
        """打开 MuJoCo viewer，使用键盘低速控制。"""
        self.reset()  # 初始化。
        cmd_state = CommandState(self.cfg)  # 创建命令状态。
        key_callback = make_key_callback(cmd_state)  # 创建键盘回调。
        sim_time = 0.0  # 仿真时间。
        last_print = 0.0  # 上次打印时间。

        print("[键盘] ↑/↓ 每次只改 0.05；先按一次 ↑，观察 5 秒，再继续加")
        print("[安全] 姿态角过大或机身太低会自动停止；空格/R 可急停")
        print("[目标] 当前阶段先要稳定低速运动，不追求速度和漂亮步态\n")

        with mujoco.viewer.launch_passive(self.model, self.data, key_callback=key_callback) as viewer:
            while viewer.is_running() and sim_time < self.cfg.duration:
                step_start = time.perf_counter()  # 用于实时限速。
                cmd = cmd_state.update(self.dt) if sim_time >= self.cfg.stand_time else np.zeros(3, dtype=np.float32)  # 站稳后才让命令生效。
                self.run_step(sim_time, cmd)  # 执行控制。
                viewer.sync()  # 刷新窗口。
                sim_time += self.dt  # 更新时间。

                if sim_time - last_print > 0.5:
                    last_print = sim_time  # 控制打印频率。
                    self.print_status(sim_time, cmd)  # 打印状态。

                if sim_time > self.cfg.stand_time + 0.5 and has_fallen(self.data):
                    self.print_status(sim_time, cmd, prefix="[安全停止]")  # 打印停止状态。
                    print("\n[原因] 机身高度过低或姿态角过大，说明当前命令/步态参数仍然太激进。")
                    break

                elapsed = time.perf_counter() - step_start  # 当前循环耗时。
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)  # 尽量保持实时速度。

        print("\n[结束] MuJoCo 低速步态实验结束")


# ============================================================
# 6. 命令行入口
# ============================================================
def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Go2 MuJoCo 低速稳定步态实验台")
    parser.add_argument("--headless-test", action="store_true", help="不打开窗口，只做稳定性自检")
    parser.add_argument("--test-vx", type=float, default=0.15, help="headless 自检 vx")
    parser.add_argument("--test-vy", type=float, default=0.0, help="headless 自检 vy")
    parser.add_argument("--test-yaw", type=float, default=0.0, help="headless 自检 yaw")
    parser.add_argument("--duration", type=float, default=120.0, help="viewer 模式最长运行时间")
    parser.add_argument("--max-thigh-amp", type=float, default=0.10, help="最大大腿摆幅")
    parser.add_argument("--base-freq", type=float, default=0.8, help="基础步频")
    return parser.parse_args()


def main() -> None:
    """程序主入口。"""
    args = parse_args()  # 读取参数。
    cfg = WalkConfig(
        duration=args.duration,
        max_thigh_amp=args.max_thigh_amp,
        base_freq=args.base_freq,
    )  # 创建配置。

    print("=" * 72)
    print("Go2 MuJoCo 低速稳定步态实验台")
    print("=" * 72)
    print(f"[模型] {cfg.scene_xml}")
    print(f"[PD] kp={cfg.kp}, kd={cfg.kd}")
    print(f"[步态] base_freq={cfg.base_freq}, max_thigh_amp={cfg.max_thigh_amp}, calf_amp={cfg.calf_amp}")
    print(f"[键盘限幅] max_vx={cfg.max_vx}, max_vy={cfg.max_vy}, max_yaw={cfg.max_yaw}, step={cfg.cmd_step}")

    runner = LowSpeedWalkRunner(cfg)  # 创建 runner。

    if args.headless_test:
        test_cmd = np.array([args.test_vx, args.test_vy, args.test_yaw], dtype=np.float32)  # 测试命令。
        ok = runner.run_headless(test_cmd)  # 运行无窗口自检。
        raise SystemExit(0 if ok else 1)  # 失败时返回非零状态。

    runner.run_viewer()  # 默认打开 viewer。


if __name__ == "__main__":
    main()
