#!/usr/bin/env python3
"""
Go2 TheoBounac RL policy -> MuJoCo Sim2Sim 安全 runner。

这个文件是 Go2 底层强化学习控制路线的第一版“安全接入器”：
  1. 加载 TheoBounac/Deploy_SimToReal_RL_Go2 提供的 Go2 TorchScript policy。
  2. 在 MuJoCo 中先用传统 PD 控制平滑站起。
  3. 严格按 TheoBounac 部署脚本拼 52 维 observation。
  4. 低频调用 policy，得到 12 维 action。
  5. 用 target_q = default_q + action * action_scale 得到目标关节角。
  6. 用高频 MuJoCo PD 内环驱动机器人。
  7. 姿态异常时进入阻尼安全态 DAMPING_STOP，而不是把目标角突然置零。

默认运行只给零速度命令，目标是先验证“站稳 + policy 接管不倒”。
不要把本文件直接用于实机；它只用于 MuJoCo Sim2Sim 学习和验证。

推荐无窗口自检：
  cd /home/ros/unitree_dev
  env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
    --headless-test

打开 MuJoCo 窗口：
  env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py

关节映射验证：
  env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
    --verify-joints --verify-joint FR_thigh_joint
"""

from __future__ import annotations

import argparse
import enum
import math
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import torch
import yaml


# ============================================================
# 1. 路径与顺序约定
# ============================================================
PROJECT_ROOT = Path(__file__).resolve().parents[2]  # 工程根目录：/home/ros/unitree_dev。
GO2_SCENE_XML = PROJECT_ROOT / "src/unitree_mujoco/unitree_robots/go2/scene.xml"  # Go2 MuJoCo 平地场景。
THEO_PROJECT = PROJECT_ROOT / "projects/open_source_deploy_simtoreal_rl_go2"  # TheoBounac Go2 RL 部署仓库。
THEO_CONFIG = THEO_PROJECT / "deploy_real/configs/go2.yaml"  # TheoBounac Go2 部署配置。
THEO_POLICY = THEO_PROJECT / "pre_train/policy_rough.pt"  # TheoBounac 默认预训练 policy。


# TheoBounac deploy_real_isaaclab.py 中 defaut_joint 的实际顺序。
# 注意：这不是 MuJoCo qpos 顺序，也不是 actuator 顺序，而是 policy 观测/action 使用的顺序。
POLICY_JOINT_NAMES = [
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
]  # policy 顺序：先 4 个 hip，再 4 个 thigh，再 4 个 calf。


POLICY_DEFAULT_Q = np.array([
    0.1, -0.1, 0.1, -0.1,
    0.8, 0.8, 1.0, 1.0,
    -1.5, -1.5, -1.5, -1.5,
], dtype=np.float64)  # policy 顺序下 action=0 时的默认关节角。


LOWSTATE_FOOT_ORDER = ["FR", "FL", "RR", "RL"]  # 宇树 Go2 LowState foot_force 常见顺序。
THEO_FOOT_OBS_FROM_LOWSTATE = [1, 3, 0, 2]  # 复刻 TheoBounac: [foot[1], foot[3], foot[0], foot[2]] / 100。


@dataclass
class TheoConfig:
    """只读取当前 runner 必须使用的 TheoBounac Go2 配置字段。"""

    policy_path: Path  # TorchScript policy 路径。
    action_scale: float  # action 到目标关节角的缩放系数，通常是 0.25。
    cmd_scale: np.ndarray  # 速度命令缩放。
    max_cmd: np.ndarray  # 速度命令最大值。
    num_obs: int  # policy 输入维度，TheoBounac Go2 policy 为 52。
    num_actions: int  # policy 输出维度，Go2 为 12。

    @classmethod
    def load(cls, config_path: Path, policy_override: Path | None) -> "TheoConfig":
        """从 go2.yaml 读取配置，并解析 policy 路径。"""
        with config_path.open("r", encoding="utf-8") as f:  # 读取 TheoBounac 的 YAML 配置。
            raw = yaml.safe_load(f)  # 使用 YAML 解析器，避免手写字符串解析。

        policy_path = policy_override if policy_override is not None else THEO_PROJECT / "pre_train" / raw["policy_path"]
        return cls(
            policy_path=policy_path,  # 最终 policy 文件路径。
            action_scale=float(raw["action_scale"]),  # TheoBounac 配置中的 action_scale。
            cmd_scale=np.asarray(raw["cmd_scale"], dtype=np.float32),  # 速度命令缩放。
            max_cmd=np.asarray(raw["max_cmd"], dtype=np.float32),  # 最大速度命令。
            num_obs=int(raw["num_obs"]),  # 应该是 52。
            num_actions=int(raw["num_actions"]),  # 应该是 12。
        )


@dataclass
class RunnerConfig:
    """MuJoCo Sim2Sim runner 的所有可调参数。"""

    scene_xml: Path = GO2_SCENE_XML  # MuJoCo 场景文件路径。
    theo_config: Path = THEO_CONFIG  # TheoBounac go2.yaml 路径。
    policy: Path | None = None  # 可选 policy 覆盖路径；None 表示使用 go2.yaml 里的 policy_path。
    sim_duration: float = 600.0 # viewer 模式最长运行时间。
    headless_duration: float = 12.0  # 无窗口自检时长。
    stand_time: float = 2.5  # PD 缓起时间。
    stabilize_time: float = 1.5  # 到达默认站姿后保持一段时间，再让 policy 接管。
    policy_dt: float = 0.02  # policy 推理周期，默认 50 Hz。
    kp: float = 40.0  # RL_CONTROL 阶段 PD 比例增益；TheoBounac 实机脚本里实际使用过 40。
    kd: float = 0.5  # RL_CONTROL 阶段 PD 阻尼；TheoBounac 实机脚本里实际使用过 0.5。
    stand_kp: float = 60.0  # PD_STAND_UP 阶段更强一点，让机器人站起。
    stand_kd: float = 5.0  # PD_STAND_UP 阶段阻尼。
    damping_kd: float = 2.0  # DAMPING_STOP 阻尼安全态的阻尼系数。
    damping_hold_kp_initial: float = 8.0  # DAMPING_STOP 初期短暂保持当前关节角的弱 kp，避免瞬间瘫软砸地。
    damping_hold_decay_time: float = 2.0  # DAMPING_STOP 中弱 kp 从初始值衰减到 0 的时间。
    action_clip: float = 3.0  # action 限幅，防止 OOD 时动作过大。
    joint_limit_margin: float = 0.04  # 目标关节角软限幅余量。
    foot_force_scale: float = 100.0  # TheoBounac 实机脚本把 foot_force 除以 100 后喂给 policy。
    foot_force_clip: float = 5.0  # MuJoCo 接触力可能尖峰很大，默认把 obs 中的足端力限制到 0~5。
    foot_force_binary: bool = False  # 是否把足端力改成 0/1 接触指示器，用于排查接触力尺度问题。
    foot_contact_threshold: float = 20.0  # 二值接触模式的牛顿阈值；TheoBounac 速度估计里也用过 20。
    fall_angle: float = math.radians(45.0)  # roll/pitch 超过 45 度进入 DAMPING_STOP。
    fall_height: float = 0.13  # base 高度过低进入 DAMPING_STOP。
    enable_keyboard_cmd: bool = False  # 默认禁止键盘速度命令，只验证零速度站稳。
    cmd_step: float = 0.05  # 键盘每次改变速度命令的幅度。
    cmd_vx: float = 0.0  # 固定 vx 命令；第一版默认 0。
    cmd_vy: float = 0.0  # 固定 vy 命令；第一版默认 0。
    cmd_yaw: float = 0.0  # 固定 yaw 命令；第一版默认 0。
    scripted_test: str = "none"  # 固定动作脚本；none 表示不用脚本，使用固定命令或键盘命令。
    turn_yaw: float = 0.25  # 掉头测试阶段的 yaw 命令，数值较大时要先在 headless 中验收。
    turn_duration: float = 12.0  # 掉头测试阶段持续时间；0.25 rad/s * 12 s 约等于 180 度命令量。
    post_turn_stop_duration: float = 2.0  # 掉头后先停住一小段时间，观察姿态是否能恢复稳定。
    forward_vx: float = 0.20  # 掉头后直行阶段的 vx 命令，比之前 0.05 的保守测试更快。
    forward_duration: float = 12.0  # 直行测试阶段持续时间。
    strafe_vy: float = 0.15  # 自动脚本左右平移阶段的 vy 命令；正数表示向左，负数表示向右。
    left_duration: float = 3.0  # 自动脚本向左平移持续时间。
    right_duration: float = 3.0  # 自动脚本向右平移持续时间。
    verify_joints: bool = False  # 是否进入关节映射验证模式。
    verify_joint: str = "FR_thigh_joint"  # 默认验证右前腿大腿关节。
    verify_amplitude: float = 0.25  # 关节验证时的目标角偏移。


class RunnerState(enum.Enum):
    """控制状态机。"""

    PD_STAND_UP = "PD_STAND_UP"  # 从 keyframe/home 平滑站到默认角。
    STABILIZE = "STABILIZE"  # 保持默认角，等待速度和姿态稳定。
    RL_CONTROL = "RL_CONTROL"  # policy 接管，低频推理 + 高频 PD。
    DAMPING_STOP = "DAMPING_STOP"  # 摔倒/异常后的阻尼安全态。


# ============================================================
# 2. 四元数、姿态与 projected gravity
# ============================================================
def quat_wxyz_to_rotmat(q_wxyz: np.ndarray) -> np.ndarray:
    """把 MuJoCo [w, x, y, z] 四元数转成旋转矩阵 R_body_to_world。"""
    q = np.asarray(q_wxyz, dtype=np.float64).copy()  # 明确复制，避免外部数据被修改。
    norm = np.linalg.norm(q)  # 四元数理论上是单位长度，但数值上仍做归一化。
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)  # 极端异常时返回单位阵，避免 NaN 继续扩散。
    w, x, y, z = q / norm  # MuJoCo free joint 四元数顺序是 [w, x, y, z]。
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)  # 返回 body -> world 的旋转矩阵。


def rotate_world_to_body(q_wxyz: np.ndarray, vec_world: np.ndarray) -> np.ndarray:
    """把世界坐标系向量旋转到机身坐标系。"""
    rot_body_to_world = quat_wxyz_to_rotmat(q_wxyz)  # R 表示 body -> world。
    return rot_body_to_world.T @ np.asarray(vec_world, dtype=np.float64)  # R^T * v_world = v_body。


def projected_gravity_body(q_wxyz: np.ndarray) -> np.ndarray:
    """计算 projected_gravity: 世界重力 [0, 0, -1] 在机身坐标系下的表达。"""
    return rotate_world_to_body(q_wxyz, np.array([0.0, 0.0, -1.0], dtype=np.float64))  # g_body。


def pitch_roll_from_quat_wxyz(q_wxyz: np.ndarray) -> tuple[float, float]:
    """从 [w, x, y, z] 四元数提取 pitch/roll，用于摔倒检测。"""
    w, x, y, z = q_wxyz  # MuJoCo free joint 四元数顺序。
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))  # 绕 y 轴俯仰角。
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))  # 绕 x 轴横滚角。
    return float(pitch), float(roll)


def smoothstep(x: float) -> float:
    """0 到 1 的平滑插值函数，比线性插值更适合 PD 缓起。"""
    x = min(1.0, max(0.0, x))  # 先限制到 [0, 1]。
    return x * x * (3.0 - 2.0 * x)  # smoothstep 曲线。


# ============================================================
# 3. MuJoCo 关节顺序映射
# ============================================================
class JointOrder:
    """集中管理 policy 顺序、MuJoCo qpos 顺序、MuJoCo actuator 顺序之间的转换。"""

    def __init__(self, model: mujoco.MjModel):
        self.model = model  # 保存 MuJoCo 模型。
        self.policy_joint_names = list(POLICY_JOINT_NAMES)  # policy 顺序下的 joint 名列表。
        self.qpos_indices = self._build_qpos_indices()  # policy index -> qpos[7:] index。
        self.qvel_indices = self._build_qvel_indices()  # policy index -> qvel[6:] index。
        self.actuator_indices = self._build_actuator_indices()  # policy index -> actuator index。
        self.qpos_to_actuator = self._build_qpos_to_actuator()  # qpos[7:] index -> actuator index。
        self.policy_joint_ranges = self._build_policy_joint_ranges()  # policy 顺序下每个关节的角度范围。

    def _joint_id(self, joint_name: str) -> int:
        """根据 joint 名找到 MuJoCo joint id。"""
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)  # 查 joint id。
        if joint_id < 0:
            raise ValueError(f"MuJoCo 模型中找不到 joint: {joint_name}")  # 名字错了就立刻失败。
        return int(joint_id)

    def _build_qpos_indices(self) -> np.ndarray:
        """构建 policy index -> qpos[7:] index。"""
        indices = []  # 保存映射。
        for name in self.policy_joint_names:
            joint_id = self._joint_id(name)  # 找 joint id。
            indices.append(int(self.model.jnt_qposadr[joint_id]) - 7)  # free joint 占 qpos 前 7 维。
        return np.asarray(indices, dtype=np.int32)  # 返回 NumPy 数组，便于高级索引。

    def _build_qvel_indices(self) -> np.ndarray:
        """构建 policy index -> qvel[6:] index。"""
        indices = []  # 保存映射。
        for name in self.policy_joint_names:
            joint_id = self._joint_id(name)  # 找 joint id。
            indices.append(int(self.model.jnt_dofadr[joint_id]) - 6)  # free joint 占 qvel 前 6 维。
        return np.asarray(indices, dtype=np.int32)  # 返回 NumPy 数组。

    def _build_actuator_indices(self) -> np.ndarray:
        """构建 policy index -> actuator index。"""
        joint_to_actuator: dict[int, int] = {}  # joint id -> actuator id。
        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id][0])  # 当前 actuator 驱动的 joint。
            joint_to_actuator[joint_id] = actuator_id  # 保存映射。

        indices = []  # 保存 policy -> actuator。
        for name in self.policy_joint_names:
            joint_id = self._joint_id(name)  # 找 joint id。
            if joint_id not in joint_to_actuator:
                raise ValueError(f"joint 没有对应 actuator: {name}")  # 当前模型必须 12 个腿关节都有 actuator。
            indices.append(joint_to_actuator[joint_id])  # 保存 actuator id。
        return np.asarray(indices, dtype=np.int32)  # 返回 NumPy 数组。

    def _build_qpos_to_actuator(self) -> np.ndarray:
        """构建 qpos[7:] index -> actuator index，用于把 qpos 顺序力矩写入 d.ctrl。"""
        qpos_to_actuator = np.zeros(self.model.nu, dtype=np.int32)  # Go2 12 个关节。
        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id][0])  # actuator 对应 joint。
            qpos_index = int(self.model.jnt_qposadr[joint_id]) - 7  # joint 对应 qpos[7:] index。
            qpos_to_actuator[qpos_index] = actuator_id  # 保存映射。
        return qpos_to_actuator  # 返回映射数组。

    def _build_policy_joint_ranges(self) -> np.ndarray:
        """读取 policy 顺序下每个关节的 MuJoCo 角度范围。"""
        ranges = np.zeros((len(self.policy_joint_names), 2), dtype=np.float64)  # 每行 [lo, hi]。
        for i, name in enumerate(self.policy_joint_names):
            joint_id = self._joint_id(name)  # 找 joint id。
            ranges[i] = self.model.jnt_range[joint_id]  # 读取 joint range。
        return ranges  # 返回 policy 顺序下的范围。

    def qpos_to_policy(self, qpos_joints: np.ndarray) -> np.ndarray:
        """把 MuJoCo qpos[7:] 顺序转换成 policy 顺序。"""
        return np.asarray(qpos_joints, dtype=np.float64)[self.qpos_indices]  # 高级索引转换。

    def qvel_to_policy(self, qvel_joints: np.ndarray) -> np.ndarray:
        """把 MuJoCo qvel[6:] 顺序转换成 policy 顺序。"""
        return np.asarray(qvel_joints, dtype=np.float64)[self.qvel_indices]  # 高级索引转换。

    def policy_to_qpos(self, policy_vec: np.ndarray) -> np.ndarray:
        """把 policy 顺序的 12 维向量转换成 MuJoCo qpos[7:] 顺序。"""
        out = np.zeros(self.model.nu, dtype=np.float64)  # MuJoCo qpos[7:] 顺序输出。
        out[self.qpos_indices] = np.asarray(policy_vec, dtype=np.float64)  # policy -> qpos。
        return out  # 返回 qpos 顺序向量。

    def policy_to_actuator(self, policy_vec: np.ndarray) -> np.ndarray:
        """把 policy 顺序的 12 维向量转换成 MuJoCo actuator 顺序。"""
        out = np.zeros(self.model.nu, dtype=np.float64)  # actuator 顺序输出。
        out[self.actuator_indices] = np.asarray(policy_vec, dtype=np.float64)  # policy -> actuator。
        return out  # 返回 actuator 顺序向量。

    def qpos_to_actuator_vec(self, qpos_vec: np.ndarray) -> np.ndarray:
        """把 qpos[7:] 顺序的 12 维向量转换成 actuator 顺序。"""
        return np.asarray(qpos_vec, dtype=np.float64)[self.qpos_to_actuator]  # qpos -> actuator。

    def soft_clip_policy_targets(self, target_policy: np.ndarray, margin: float) -> np.ndarray:
        """对 policy 顺序的目标关节角做软限幅，避免贴到机械限位。"""
        target = np.asarray(target_policy, dtype=np.float64).copy()  # 复制输入，避免原地修改。
        for i, (lo, hi) in enumerate(self.policy_joint_ranges):
            safe_margin = min(float(margin), max(0.0, 0.03 * float(hi - lo)))  # 防止 margin 大过可动范围。
            target[i] = np.clip(target[i], lo + safe_margin, hi - safe_margin)  # 限制目标角。
        return target  # 返回限幅后的目标角。

    def print_mapping(self) -> None:
        """打印当前三套关节顺序映射，便于人工检查。"""
        print("\n[关节映射] policy index -> joint / qpos[7:] / actuator")
        for i, name in enumerate(self.policy_joint_names):
            print(
                f"  {i:02d}: {name:<16s} "
                f"qpos_index={int(self.qpos_indices[i]):02d} "
                f"qvel_index={int(self.qvel_indices[i]):02d} "
                f"actuator={int(self.actuator_indices[i]):02d}"
            )


# ============================================================
# 4. 命令输入
# ============================================================
class CommandState:
    """保存速度命令；默认零速度，只有显式启用键盘时才响应按键。"""

    def __init__(self, cfg: RunnerConfig, theo: TheoConfig):
        self.cfg = cfg  # 保存 runner 配置。
        self.theo = theo  # 保存 Theo 配置，用于限幅。
        self.cmd = np.array([cfg.cmd_vx, cfg.cmd_vy, cfg.cmd_yaw], dtype=np.float32)  # 初始固定命令。
        self._clip()  # 启动时先限幅一次。

    def _clip(self) -> None:
        """按 Theo 配置的 max_cmd 限制速度命令。"""
        self.cmd[0] = np.clip(self.cmd[0], -self.theo.max_cmd[0], self.theo.max_cmd[0])  # vx 限幅。
        self.cmd[1] = np.clip(self.cmd[1], -self.theo.max_cmd[1], self.theo.max_cmd[1])  # vy 限幅。
        self.cmd[2] = np.clip(self.cmd[2], -self.theo.max_cmd[2], self.theo.max_cmd[2])  # yaw 限幅。

    def get(self) -> np.ndarray:
        """返回当前速度命令副本。"""
        return self.cmd.copy()  # 返回副本，避免外部直接修改内部状态。

    def stop(self) -> None:
        """速度命令清零。"""
        self.cmd[:] = 0.0  # 三个速度维度全部清零。
        print("\r[cmd] stop vx=+0.00 vy=+0.00 yaw=+0.00        ", end="", flush=True)

    def add(self, dvx: float, dvy: float, dyaw: float) -> None:
        """键盘增量调整速度命令。"""
        if not self.cfg.enable_keyboard_cmd:
            print("\r[cmd] keyboard disabled, use --enable-keyboard-cmd to unlock commands.        ", end="", flush=True)
            return
        self.cmd += np.array([dvx, dvy, dyaw], dtype=np.float32)  # 增量修改命令。
        self._clip()  # 修改后立刻限幅。
        print(
            f"\r[cmd] vx={self.cmd[0]:+.2f} vy={self.cmd[1]:+.2f} yaw={self.cmd[2]:+.2f}        ",
            end="",
            flush=True,
        )


def make_key_callback(cmd_state: CommandState):
    """创建 MuJoCo viewer 键盘回调。"""
    step = cmd_state.cfg.cmd_step  # 每次按键的速度增量。

    def callback(keycode: int) -> None:
        from mujoco.glfw import glfw  # 延迟导入，避免 headless 模式初始化 GLFW。

        if keycode == glfw.KEY_UP:
            cmd_state.add(+step, 0.0, 0.0)  # ↑：增加前进速度。
        elif keycode == glfw.KEY_DOWN:
            cmd_state.add(-step, 0.0, 0.0)  # ↓：增加后退速度。
        elif keycode == glfw.KEY_LEFT:
            cmd_state.add(0.0, +step, 0.0)  # ←：增加左移速度。
        elif keycode == glfw.KEY_RIGHT:
            cmd_state.add(0.0, -step, 0.0)  # →：增加右移速度。
        elif keycode == glfw.KEY_Q:
            cmd_state.add(0.0, 0.0, +step)  # Q：增加左转速度。
        elif keycode == glfw.KEY_E:
            cmd_state.add(0.0, 0.0, -step)  # E：增加右转速度。
        elif keycode in (glfw.KEY_SPACE, glfw.KEY_R):
            cmd_state.stop()  # 空格/R：速度命令清零。

    return callback  # 返回 viewer 可用的回调函数。


# ============================================================
# 5. 主 runner
# ============================================================
class Go2MujocoRlPolicyRunner:
    """TheoBounac Go2 RL policy 在 MuJoCo 中的安全 Sim2Sim runner。"""

    def __init__(self, cfg: RunnerConfig):
        self.cfg = cfg  # 保存运行配置。
        self.theo = TheoConfig.load(cfg.theo_config, cfg.policy)  # 读取 TheoBounac 配置。

        if self.theo.num_obs != 52:
            raise ValueError(f"TheoBounac Go2 policy 期望 52 维 obs，当前配置是 {self.theo.num_obs}")  # 防止错策略。
        if self.theo.num_actions != 12:
            raise ValueError(f"Go2 policy 应输出 12 维 action，当前配置是 {self.theo.num_actions}")  # 防止错策略。
        if not cfg.scene_xml.exists():
            raise FileNotFoundError(f"找不到 MuJoCo 场景: {cfg.scene_xml}")  # 场景文件必须存在。
        if not self.theo.policy_path.exists():
            raise FileNotFoundError(f"找不到 TorchScript policy: {self.theo.policy_path}")  # policy 必须存在。

        self.model = mujoco.MjModel.from_xml_path(str(cfg.scene_xml))  # 加载 MuJoCo Go2 模型。
        self.data = mujoco.MjData(self.model)  # 创建仿真数据。
        self.dt = float(self.model.opt.timestep)  # MuJoCo 物理步长，go2.xml 当前是 0.002。
        self.policy_decimation = max(1, int(round(cfg.policy_dt / self.dt)))  # policy_dt / sim_dt 得到底层步数。

        self.joints = JointOrder(self.model)  # 建立三套关节顺序映射。
        self.policy = torch.jit.load(str(self.theo.policy_path), map_location="cpu")  # 加载 TorchScript policy。
        self.policy.eval()  # 设置为推理模式。

        self.default_policy = POLICY_DEFAULT_Q.copy()  # policy 顺序默认角。
        self.default_qpos = self.joints.policy_to_qpos(self.default_policy)  # 转成 MuJoCo qpos[7:] 顺序。
        self.start_qpos = np.zeros(12, dtype=np.float64)  # reset 后的起始关节角。
        self.target_policy = self.default_policy.copy()  # policy 顺序的当前目标角。
        self.target_qpos = self.default_qpos.copy()  # qpos 顺序的当前目标角。
        self.prev_action = np.zeros(12, dtype=np.float32)  # 上一帧 policy action，obs[40:52] 必须用它。
        self.state = RunnerState.PD_STAND_UP  # 初始状态。
        self.state_start_time = 0.0  # 当前状态开始时间。
        self.step_count = 0  # MuJoCo 物理步计数。
        self.max_abs_action = 0.0  # 记录最大 action，便于判断策略是否饱和。
        self.clip_count = 0  # action 被限幅的次数。
        self.damping_steps = 0  # DAMPING_STOP 已执行步数。
        self.damping_hold_qpos = np.zeros(12, dtype=np.float64)  # 进入 DAMPING_STOP 瞬间锁定的关节角。
        self.last_cmd = np.zeros(3, dtype=np.float32)  # 最近一次送入 policy obs 的速度命令，便于打印脚本状态。

        self.foot_geom_ids = self._find_foot_geom_ids()  # 找到 FL/FR/RL/RR 足端碰撞 geom。

    def _find_foot_geom_ids(self) -> dict[str, int]:
        """查找 MuJoCo 中名为 FL/FR/RL/RR 的足端 geom。"""
        foot_geom_ids: dict[str, int] = {}  # leg name -> geom id。
        for leg in ["FL", "FR", "RL", "RR"]:
            geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, leg)  # 查找足端 geom。
            if geom_id >= 0:
                foot_geom_ids[leg] = int(geom_id)  # 保存找到的 geom id。
        return foot_geom_ids  # 返回足端 geom 映射。

    def reset(self) -> None:
        """重置仿真到 MuJoCo keyframe home，并初始化状态机。"""
        mujoco.mj_resetData(self.model, self.data)  # 先让 MuJoCo 清空所有状态。
        if self.model.nkey > 0:
            self.data.qpos[:] = self.model.key_qpos[0].copy()  # 使用 go2.xml 的 home keyframe。
            self.data.qvel[:] = 0.0  # 初始速度清零。
        else:
            self.data.qpos[:7] = [0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0]  # 兜底 free joint。
            self.data.qpos[7:19] = self.default_qpos.copy()  # 兜底关节角。
            self.data.qvel[:] = 0.0  # 初始速度清零。

        mujoco.mj_forward(self.model, self.data)  # 同步派生量，保证第一帧传感器/位置有效。
        self.start_qpos = self.data.qpos[7:19].copy()  # 记录 PD 缓起起点。
        self.target_qpos = self.start_qpos.copy()  # 初始目标等于当前角。
        self.target_policy = self.joints.qpos_to_policy(self.target_qpos)  # 同步 policy 顺序目标。
        self.prev_action[:] = 0.0  # policy 历史动作清零。
        self.data.ctrl[:] = 0.0  # 初始力矩清零。
        self.state = RunnerState.PD_STAND_UP  # 从 PD 缓起开始。
        self.state_start_time = 0.0  # 当前状态开始时间。
        self.step_count = 0  # 物理步计数清零。
        self.max_abs_action = 0.0  # 统计清零。
        self.clip_count = 0  # 统计清零。
        self.damping_steps = 0  # 阻尼步数清零。
        self.damping_hold_qpos = self.data.qpos[7:19].copy()  # 初始化阻尼保持目标，防止异常状态未赋值。
        self.last_cmd[:] = 0.0  # 重置后命令清零。

    def _set_state(self, new_state: RunnerState, sim_time: float) -> None:
        """切换状态机，并记录状态开始时间。"""
        if self.state != new_state:
            print(f"\n[state] {self.state.value} -> {new_state.value} at t={sim_time:.2f}s")
            if new_state == RunnerState.DAMPING_STOP:
                self.damping_hold_qpos = self.data.qpos[7:19].copy()  # 记录触发瞬间的关节角，用于弱保持再衰减。
                self.damping_steps = 0  # 每次进入安全态都重新统计阻尼步数。
            self.state = new_state  # 更新状态。
            self.state_start_time = sim_time  # 记录切换时间。

    def _base_pitch_roll(self) -> tuple[float, float]:
        """读取 base 当前 pitch/roll。"""
        return pitch_roll_from_quat_wxyz(self.data.qpos[3:7])  # 从 free joint 四元数计算。

    def _fallen(self) -> bool:
        """判断是否已经进入危险姿态。"""
        pitch, roll = self._base_pitch_roll()  # 读取姿态。
        return bool(
            self.data.qpos[2] < self.cfg.fall_height
            or abs(pitch) > self.cfg.fall_angle
            or abs(roll) > self.cfg.fall_angle
        )  # 高度太低或姿态角太大都认为危险。

    def _foot_force_lowstate_order(self) -> np.ndarray:
        """从 MuJoCo contact 估计足端法向力，并输出 LowState 常见顺序 [FR, FL, RR, RL]。"""
        force_by_leg = {leg: 0.0 for leg in ["FR", "FL", "RR", "RL"]}  # 初始化每条腿足端力。
        geom_to_leg = {geom_id: leg for leg, geom_id in self.foot_geom_ids.items()}  # geom id -> leg name。
        contact_force = np.zeros(6, dtype=np.float64)  # mj_contactForce 输出 6 维接触力/力矩。

        for contact_id in range(self.data.ncon):
            contact = self.data.contact[contact_id]  # 当前接触。
            leg = geom_to_leg.get(int(contact.geom1)) or geom_to_leg.get(int(contact.geom2))  # 判断是否足端接触。
            if leg is None:
                continue
            mujoco.mj_contactForce(self.model, self.data, contact_id, contact_force)  # 读取接触坐标系下力。
            force_by_leg[leg] += max(0.0, float(contact_force[0]))  # contact_force[0] 是法向力。

        return np.array([force_by_leg[leg] for leg in LOWSTATE_FOOT_ORDER], dtype=np.float32)  # 输出 LowState 顺序。

    def _foot_force_obs_lowstate_order(self, foot_lowstate: np.ndarray) -> np.ndarray:
        """把 MuJoCo 原始足端力转换成 policy obs 使用的足端力特征，仍保持 LowState 顺序。"""
        if self.cfg.foot_force_binary:
            return (foot_lowstate >= self.cfg.foot_contact_threshold).astype(np.float32)  # 0/1 接触指示器，方便排查策略是否更依赖接触状态。
        scaled = foot_lowstate / max(self.cfg.foot_force_scale, 1e-6)  # 默认复刻 TheoBounac: foot_force / 100。
        return np.clip(scaled, 0.0, self.cfg.foot_force_clip).astype(np.float32)  # 限制 MuJoCo 接触尖峰，避免 obs 被异常大力污染。

    def _build_obs(self, cmd_vel: np.ndarray) -> np.ndarray:
        """严格复刻 TheoBounac 预训练 policy 的 52 维 observation。"""
        quat = self.data.qpos[3:7].copy()  # MuJoCo free joint 四元数 [w, x, y, z]。
        q_policy = self.joints.qpos_to_policy(self.data.qpos[7:19])  # 当前关节角转 policy 顺序。
        dq_policy = self.joints.qvel_to_policy(self.data.qvel[6:18])  # 当前关节速度转 policy 顺序。
        foot_lowstate = self._foot_force_lowstate_order()  # MuJoCo 足端力估计，LowState 顺序。
        foot_obs_lowstate = self._foot_force_obs_lowstate_order(foot_lowstate)  # 转成 policy 需要的足端力特征。

        obs = np.zeros(52, dtype=np.float32)  # TheoBounac Go2 policy 输入维度固定为 52。
        obs[0:4] = foot_obs_lowstate[THEO_FOOT_OBS_FROM_LOWSTATE]  # 复刻 [foot[1], foot[3], foot[0], foot[2]] 的顺序。
        obs[4:7] = rotate_world_to_body(quat, self.data.qvel[0:3]).astype(np.float32)  # base_lin_vel，机身坐标系。
        obs[7:10] = self.data.qvel[3:6].astype(np.float32)  # MuJoCo freejoint 角速度已经是机身局部表达，等价于 IMU gyro，不再旋转。
        obs[10:13] = projected_gravity_body(quat).astype(np.float32)  # projected_gravity。
        obs[13:16] = (cmd_vel.astype(np.float32) * self.theo.cmd_scale * self.theo.max_cmd).astype(np.float32)  # 速度命令。
        obs[16:28] = (q_policy - self.default_policy).astype(np.float32)  # q - default_q。
        obs[28:40] = dq_policy.astype(np.float32)  # dq。
        obs[40:52] = self.prev_action.astype(np.float32)  # 上一帧 policy action，不是 PD ctrl。
        return obs  # 返回 52 维观测。

    def _policy_step(self, cmd_vel: np.ndarray) -> None:
        """低频 policy 推理：obs -> action -> target_q。"""
        obs = self._build_obs(cmd_vel)  # 拼 52 维 observation。
        obs_tensor = torch.from_numpy(obs).unsqueeze(0).float()  # 添加 batch 维度。
        with torch.no_grad():
            action = self.policy(obs_tensor).detach().cpu().numpy().reshape(-1)  # 推理得到 12 维 action。

        if action.shape[0] != 12:
            raise RuntimeError(f"policy 输出维度不是 12: {action.shape}")  # 防止错模型继续运行。

        raw_max = float(np.max(np.abs(action)))  # 记录原始动作幅度。
        clipped_action = np.clip(action, -self.cfg.action_clip, self.cfg.action_clip)  # action 限幅。
        if raw_max > self.cfg.action_clip:
            self.clip_count += 1  # 统计限幅次数。
        self.max_abs_action = max(self.max_abs_action, raw_max)  # 更新最大动作。

        target_policy = self.default_policy + clipped_action * self.theo.action_scale  # Target_Q = Default_Q + Action * Action_Scale。
        target_policy = self.joints.soft_clip_policy_targets(target_policy, self.cfg.joint_limit_margin)  # 目标角软限幅。
        self.target_policy = target_policy  # 保存 policy 顺序目标。
        self.target_qpos = self.joints.policy_to_qpos(target_policy)  # 转成 MuJoCo qpos 顺序。
        self.prev_action = clipped_action.astype(np.float32)  # 本轮结束后保存为下一帧 last_action。

    def _pd_torque_qpos(self, target_qpos: np.ndarray, kp: float, kd: float) -> np.ndarray:
        """计算 qpos 顺序的 PD 力矩。"""
        q = self.data.qpos[7:19]  # 当前 12 个关节角，qpos 顺序。
        dq = self.data.qvel[6:18]  # 当前 12 个关节速度，qvel 顺序。
        return kp * (target_qpos - q) - kd * dq  # tau = kp * (q_des - q) - kd * dq。

    def _apply_torque_qpos(self, tau_qpos: np.ndarray) -> None:
        """把 qpos 顺序力矩转换到 actuator 顺序、限幅后写入 data.ctrl。"""
        tau_actuator = self.joints.qpos_to_actuator_vec(tau_qpos)  # qpos 顺序 -> actuator 顺序。
        for i in range(self.model.nu):
            lo, hi = self.model.actuator_ctrlrange[i]  # 读取该 actuator 力矩范围。
            self.data.ctrl[i] = np.clip(tau_actuator[i], lo, hi)  # 写入限幅后的力矩。

    def _pd_step(self, target_qpos: np.ndarray, kp: float, kd: float) -> None:
        """执行一步 PD 控制并推进 MuJoCo。"""
        tau_qpos = self._pd_torque_qpos(target_qpos, kp, kd)  # 计算 qpos 顺序力矩。
        self._apply_torque_qpos(tau_qpos)  # 写入 actuator ctrl。
        mujoco.mj_step(self.model, self.data)  # 推进一个物理步。
        self.step_count += 1  # 物理步计数递增。

    def _damping_step(self, sim_time: float) -> None:
        """阻尼安全态：先弱保持当前关节角，再逐步衰减为纯阻尼。"""
        elapsed = max(0.0, sim_time - self.state_start_time)  # 进入 DAMPING_STOP 后经过的时间。
        decay = max(0.0, 1.0 - elapsed / max(self.cfg.damping_hold_decay_time, 1e-6))  # 线性衰减系数。
        hold_kp = self.cfg.damping_hold_kp_initial * decay  # 初期弱保持，随后逐渐衰减到 0。
        q = self.data.qpos[7:19]  # 当前 12 个关节角。
        dq = self.data.qvel[6:18]  # 当前 12 个关节速度。
        tau_qpos = hold_kp * (self.damping_hold_qpos - q) - self.cfg.damping_kd * dq  # 弱位置保持 + 粘滞阻尼。
        self._apply_torque_qpos(tau_qpos)  # 写入 actuator ctrl。
        mujoco.mj_step(self.model, self.data)  # 推进仿真。
        self.step_count += 1  # 物理步计数。
        self.damping_steps += 1  # 阻尼步数统计。

    def _standup_target(self, sim_time: float) -> np.ndarray:
        """PD 缓起目标：从当前初始关节角平滑插值到 policy 默认角。"""
        phase = smoothstep((sim_time - self.state_start_time) / max(self.cfg.stand_time, 1e-6))  # 站起进度。
        return (1.0 - phase) * self.start_qpos + phase * self.default_qpos  # qpos 顺序目标角。

    def _script_total_duration(self) -> float:
        """计算固定动作脚本完整跑完需要的总仿真时间。"""
        if self.cfg.scripted_test == "turn_then_forward":
            return (
                self.cfg.stand_time
                + self.cfg.stabilize_time
                + self.cfg.turn_duration
                + self.cfg.post_turn_stop_duration
                + self.cfg.forward_duration
                + 1.0
            )  # 最后多留 1 秒停稳观察。
        if self.cfg.scripted_test == "turn_forward_strafe":
            return (
                self.cfg.stand_time
                + self.cfg.stabilize_time
                + self.cfg.turn_duration
                + self.cfg.forward_duration
                + self.cfg.left_duration
                + self.cfg.right_duration
                + 1.0
            )  # 掉头、直行、左移、右移之后多留 1 秒停稳观察。
        return 0.0  # 没有脚本时不需要额外时间。

    def _command_for_time(self, sim_time: float, base_cmd: np.ndarray) -> np.ndarray:
        """根据当前时间生成速度命令；脚本模式优先，非脚本模式使用固定命令或键盘命令。"""
        if self.cfg.scripted_test == "none":
            cmd = np.asarray(base_cmd, dtype=np.float32).copy()  # 普通模式：直接使用外部传入命令。
        elif self.cfg.scripted_test == "turn_then_forward":
            cmd = np.zeros(3, dtype=np.float32)  # 脚本模式默认停住。
            if self.state == RunnerState.RL_CONTROL:
                t = sim_time - self.state_start_time  # 脚本从 RL_CONTROL 接管那一刻开始计时。
                if t < self.cfg.turn_duration:
                    cmd[2] = self.cfg.turn_yaw  # 阶段 1：原地掉头。
                elif t < self.cfg.turn_duration + self.cfg.post_turn_stop_duration:
                    cmd[:] = 0.0  # 阶段 2：短暂停住，观察姿态恢复。
                elif t < self.cfg.turn_duration + self.cfg.post_turn_stop_duration + self.cfg.forward_duration:
                    cmd[0] = self.cfg.forward_vx  # 阶段 3：较快直行。
        elif self.cfg.scripted_test == "turn_forward_strafe":
            cmd = np.zeros(3, dtype=np.float32)  # 脚本模式默认停住。
            if self.state == RunnerState.RL_CONTROL:
                t = sim_time - self.state_start_time  # 脚本从 RL_CONTROL 接管那一刻开始计时。
                turn_end = self.cfg.turn_duration  # 阶段 1 结束时间。
                forward_end = turn_end + self.cfg.forward_duration  # 阶段 2 结束时间。
                left_end = forward_end + self.cfg.left_duration  # 阶段 3 结束时间。
                right_end = left_end + self.cfg.right_duration  # 阶段 4 结束时间。
                if t < turn_end:
                    cmd[2] = self.cfg.turn_yaw  # 阶段 1：原地掉头。
                elif t < forward_end:
                    cmd[0] = self.cfg.forward_vx  # 阶段 2：直行。
                elif t < left_end:
                    cmd[1] = self.cfg.strafe_vy  # 阶段 3：向左平移。
                elif t < right_end:
                    cmd[1] = -self.cfg.strafe_vy  # 阶段 4：向右平移。
        else:
            raise ValueError(f"未知 scripted_test: {self.cfg.scripted_test}")  # 防止参数拼错后静默运行。
        self.last_cmd = cmd.astype(np.float32)  # 保存最近命令，便于状态打印。
        return self.last_cmd.copy()  # 返回副本，避免调用者修改内部状态。

    def step(self, sim_time: float, cmd_vel: np.ndarray) -> None:
        """执行一个 MuJoCo 物理步，根据状态机选择控制模式。"""
        if self.state == RunnerState.PD_STAND_UP:
            self.target_qpos = self._standup_target(sim_time)  # 计算平滑站起目标。
            self._pd_step(self.target_qpos, self.cfg.stand_kp, self.cfg.stand_kd)  # 用较强 PD 站起。
            if sim_time - self.state_start_time >= self.cfg.stand_time:
                self._set_state(RunnerState.STABILIZE, sim_time)  # 站起完成后进入稳定等待。
            return

        if self.state == RunnerState.STABILIZE:
            self.target_qpos = self.default_qpos.copy()  # 稳定阶段保持 policy 默认角。
            self._pd_step(self.target_qpos, self.cfg.stand_kp, self.cfg.stand_kd)  # 继续使用站立 PD。
            if self._fallen():
                self._set_state(RunnerState.DAMPING_STOP, sim_time)  # 站立阶段异常也进入阻尼态。
            elif sim_time - self.state_start_time >= self.cfg.stabilize_time:
                self.prev_action[:] = 0.0  # policy 接管前 last_action 清零。
                self._set_state(RunnerState.RL_CONTROL, sim_time)  # 进入 RL 控制。
            return

        if self.state == RunnerState.RL_CONTROL:
            if self._fallen():
                self._set_state(RunnerState.DAMPING_STOP, sim_time)  # RL 阶段摔倒，立即阻尼停止。
                self._damping_step(sim_time)  # 本步就进入阻尼。
                return
            if self.step_count % self.policy_decimation == 0:
                self._policy_step(cmd_vel)  # 每 policy_decimation 个物理步推理一次 policy。
            self._pd_step(self.target_qpos, self.cfg.kp, self.cfg.kd)  # 高频 PD 内环。
            return

        if self.state == RunnerState.DAMPING_STOP:
            self._damping_step(sim_time)  # 阻尼安全态执行弱保持并逐步衰减到纯阻尼。
            return

    def run_headless(self) -> bool:
        """无窗口安全自检，返回 True 表示没有进入 DAMPING_STOP。"""
        self.reset()  # 重置仿真。
        base_cmd = np.array([self.cfg.cmd_vx, self.cfg.cmd_vy, self.cfg.cmd_yaw], dtype=np.float32)  # 固定速度命令。
        sim_time = 0.0  # 当前仿真时间。
        effective_duration = max(self.cfg.headless_duration, self._script_total_duration())  # 脚本模式自动延长到完整跑完。
        total_steps = int(effective_duration / self.dt)  # 需要执行的物理步数。
        if self.cfg.scripted_test != "none" and effective_duration > self.cfg.headless_duration:
            print(f"[脚本] headless_duration 自动延长到 {effective_duration:.1f}s，以便完整执行 {self.cfg.scripted_test}。")

        for _ in range(total_steps):
            cmd_vel = self._command_for_time(sim_time, base_cmd)  # 根据脚本或固定命令得到当前速度命令。
            self.step(sim_time, cmd_vel)  # 执行一个物理步。
            sim_time += self.dt  # 更新时间。
            if self.state == RunnerState.DAMPING_STOP:
                self.print_summary(sim_time, prefix="[失败]")  # 打印失败信息。
                return False

        self.print_summary(sim_time, prefix="[通过]")  # 打印通过信息。
        return True  # 没有触发 DAMPING_STOP 即通过。

    def run_viewer(self) -> None:
        """打开 MuJoCo viewer 运行。"""
        self.reset()  # 重置仿真。
        self.joints.print_mapping()  # 启动时打印映射，便于人工确认。
        cmd_state = CommandState(self.cfg, self.theo)  # 创建速度命令状态。
        key_callback = make_key_callback(cmd_state)  # 创建键盘回调。
        sim_time = 0.0  # 当前仿真时间。
        last_print = 0.0  # 上次状态打印时间。

        print("\n[运行] 默认零速度 policy 接管；键盘速度命令默认禁用。")
        if self.cfg.scripted_test != "none":
            print(
                "[脚本] turn_then_forward: "
                f"yaw={self.cfg.turn_yaw:+.2f} 持续 {self.cfg.turn_duration:.1f}s，"
                f"停 {self.cfg.post_turn_stop_duration:.1f}s，"
                f"vx={self.cfg.forward_vx:+.2f} 持续 {self.cfg.forward_duration:.1f}s。"
            )
        if self.cfg.enable_keyboard_cmd:
            print("[键盘] ↑/↓ 前后，←/→ 左右，Q/E 转向，空格/R 清零。")
        else:
            print("[键盘] 速度命令锁定；需要键盘控制时加 --enable-keyboard-cmd。")
        print("[安全] 姿态异常会进入 DAMPING_STOP 阻尼安全态。\n")

        with mujoco.viewer.launch_passive(self.model, self.data, key_callback=key_callback) as viewer:
            effective_duration = max(self.cfg.sim_duration, self._script_total_duration())  # 脚本模式确保 viewer 不会提前结束。
            while viewer.is_running() and sim_time < effective_duration:
                step_start = time.perf_counter()  # 用于实时限速。
                cmd_vel = self._command_for_time(sim_time, cmd_state.get())  # 根据脚本或键盘得到当前速度命令。
                self.step(sim_time, cmd_vel)  # 执行控制。
                viewer.sync()  # 刷新 viewer。
                sim_time += self.dt  # 更新时间。

                if sim_time - last_print >= 1.0:
                    last_print = sim_time  # 控制打印频率。
                    self.print_status(sim_time)  # 打印状态。

                if self.state == RunnerState.DAMPING_STOP and self.damping_steps > int(3.0 / self.dt):
                    self.print_summary(sim_time, prefix="[阻尼停止完成]")  # 阻尼 3 秒后退出。
                    break

                elapsed = time.perf_counter() - step_start  # 计算本循环耗时。
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)  # 尽量按实时速度播放。

        self.print_summary(sim_time, prefix="[结束]")  # viewer 退出后打印总结。

    def run_verify_joints(self) -> None:
        """关节映射验证模式：只对指定 joint 加小偏移，不接入 policy。"""
        if self.cfg.verify_joint not in POLICY_JOINT_NAMES:
            valid = ", ".join(POLICY_JOINT_NAMES)  # 可用 joint 名。
            raise ValueError(f"--verify-joint 必须是 policy joint 名之一，当前可用：{valid}")

        self.reset()  # 重置仿真。
        self.joints.print_mapping()  # 打印映射。
        joint_index = POLICY_JOINT_NAMES.index(self.cfg.verify_joint)  # 找 policy 顺序下的 joint index。
        sim_time = 0.0  # 当前仿真时间。
        print(
            f"\n[关节验证] 当前只给 {self.cfg.verify_joint} 增加 "
            f"{self.cfg.verify_amplitude:+.2f} rad 偏移。请观察 MuJoCo 中是否为预期关节在动。\n"
        )

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and sim_time < self.cfg.sim_duration:
                step_start = time.perf_counter()  # 实时限速起点。
                if sim_time < self.cfg.stand_time:
                    target_qpos = self._standup_target(sim_time)  # 先正常站起。
                    self._pd_step(target_qpos, self.cfg.stand_kp, self.cfg.stand_kd)  # 执行站起 PD。
                else:
                    target_policy = self.default_policy.copy()  # 从默认角开始。
                    phase = 0.5 + 0.5 * math.sin(2.0 * math.pi * 0.4 * (sim_time - self.cfg.stand_time))  # 缓慢摆动。
                    target_policy[joint_index] += self.cfg.verify_amplitude * phase  # 只动一个 policy joint。
                    target_policy = self.joints.soft_clip_policy_targets(target_policy, self.cfg.joint_limit_margin)  # 限幅。
                    self.target_qpos = self.joints.policy_to_qpos(target_policy)  # 转 qpos 顺序。
                    self._pd_step(self.target_qpos, self.cfg.stand_kp, self.cfg.stand_kd)  # PD 执行。
                viewer.sync()  # 刷新窗口。
                sim_time += self.dt  # 更新时间。
                elapsed = time.perf_counter() - step_start  # 当前循环耗时。
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)  # 维持实时。

    def print_status(self, sim_time: float) -> None:
        """打印一行运行状态。"""
        pitch, roll = self._base_pitch_roll()  # 当前姿态。
        print(
            f"\r[t={sim_time:5.1f}s state={self.state.value:<12s} h={self.data.qpos[2]:.3f} "
            f"pitch={pitch:+.2f} roll={roll:+.2f} "
            f"cmd=({self.last_cmd[0]:+.2f},{self.last_cmd[1]:+.2f},{self.last_cmd[2]:+.2f}) "
            f"max|a|={self.max_abs_action:.2f} clips={self.clip_count}]",
            end="",
            flush=True,
        )

    def print_summary(self, sim_time: float, prefix: str) -> None:
        """打印最终总结。"""
        pitch, roll = self._base_pitch_roll()  # 当前姿态。
        print(
            f"\n{prefix} t={sim_time:.2f}s, state={self.state.value}, height={self.data.qpos[2]:.3f}m, "
            f"pitch={pitch:+.3f}, roll={roll:+.3f}, "
            f"last_cmd=({self.last_cmd[0]:+.3f},{self.last_cmd[1]:+.3f},{self.last_cmd[2]:+.3f}), "
            f"max|action|={self.max_abs_action:.3f}, clips={self.clip_count}, "
            f"policy_decimation={self.policy_decimation}"
        )


# ============================================================
# 6. 命令行入口
# ============================================================
def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Go2 TheoBounac RL policy -> MuJoCo Sim2Sim runner")
    parser.add_argument("--scene-xml", type=Path, default=GO2_SCENE_XML, help="MuJoCo Go2 场景 XML 路径")
    parser.add_argument("--theo-config", type=Path, default=THEO_CONFIG, help="TheoBounac go2.yaml 路径")
    parser.add_argument("--policy", type=Path, default=None, help="TorchScript policy 路径；默认读取 go2.yaml")
    parser.add_argument("--headless-test", action="store_true", help="无窗口自检，只验证零速度站稳")
    parser.add_argument("--duration", type=float, default=60.0, help="viewer 模式运行时长")
    parser.add_argument("--headless-duration", type=float, default=12.0, help="headless 自检时长")
    parser.add_argument("--stand-time", type=float, default=2.5, help="PD 缓起时间")
    parser.add_argument("--stabilize-time", type=float, default=1.5, help="policy 接管前稳定等待时间")
    parser.add_argument("--policy-dt", type=float, default=0.02, help="policy 推理周期")
    parser.add_argument("--kp", type=float, default=40.0, help="RL_CONTROL 阶段 PD kp")
    parser.add_argument("--kd", type=float, default=0.5, help="RL_CONTROL 阶段 PD kd")
    parser.add_argument("--stand-kp", type=float, default=60.0, help="PD_STAND_UP/STABILIZE 阶段 kp")
    parser.add_argument("--stand-kd", type=float, default=5.0, help="PD_STAND_UP/STABILIZE 阶段 kd")
    parser.add_argument("--damping-kd", type=float, default=2.0, help="DAMPING_STOP 阻尼 kd")
    parser.add_argument("--damping-hold-kp-initial", type=float, default=8.0, help="DAMPING_STOP 初期弱保持 kp")
    parser.add_argument("--damping-hold-decay-time", type=float, default=2.0, help="DAMPING_STOP 弱保持 kp 衰减时间")
    parser.add_argument("--action-clip", type=float, default=3.0, help="policy action 限幅")
    parser.add_argument("--foot-force-scale", type=float, default=100.0, help="足端力进入 obs 前的除数，默认复刻 foot_force/100")
    parser.add_argument("--foot-force-clip", type=float, default=5.0, help="足端力 obs 上限，避免 MuJoCo 接触尖峰")
    parser.add_argument("--foot-force-binary", action="store_true", help="把足端力改为 0/1 接触指示器，用于排查接触力尺度")
    parser.add_argument("--foot-contact-threshold", type=float, default=20.0, help="二值足端接触阈值，单位近似 N")
    parser.add_argument("--cmd-vx", type=float, default=0.0, help="固定 vx 命令")
    parser.add_argument("--cmd-vy", type=float, default=0.0, help="固定 vy 命令")
    parser.add_argument("--cmd-yaw", type=float, default=0.0, help="固定 yaw 命令")
    parser.add_argument(
        "--scripted-test",
        choices=["none", "turn_then_forward", "turn_forward_strafe"],
        default="none",
        help="固定动作脚本；turn_then_forward 表示先掉头再直行，turn_forward_strafe 表示掉头、直行、左移、右移",
    )
    parser.add_argument("--turn-yaw", type=float, default=0.25, help="自动脚本掉头阶段 yaw 命令")
    parser.add_argument("--turn-duration", type=float, default=12.0, help="自动脚本掉头阶段持续时间")
    parser.add_argument("--post-turn-stop-duration", type=float, default=2.0, help="turn_then_forward 掉头后停顿时间")
    parser.add_argument("--forward-vx", type=float, default=0.20, help="自动脚本直行阶段 vx 命令")
    parser.add_argument("--forward-duration", type=float, default=12.0, help="自动脚本直行阶段持续时间")
    parser.add_argument("--strafe-vy", type=float, default=0.15, help="turn_forward_strafe 左右平移阶段 vy 命令幅度")
    parser.add_argument("--left-duration", type=float, default=3.0, help="turn_forward_strafe 向左平移持续时间")
    parser.add_argument("--right-duration", type=float, default=3.0, help="turn_forward_strafe 向右平移持续时间")
    parser.add_argument("--enable-keyboard-cmd", action="store_true", help="打开 viewer 键盘速度命令")
    parser.add_argument("--cmd-step", type=float, default=0.05, help="键盘每次改变的速度命令幅度")
    parser.add_argument("--verify-joints", action="store_true", help="进入关节映射验证模式，不加载 policy 控制")
    parser.add_argument("--verify-joint", default="FR_thigh_joint", help="关节验证模式下要单独摆动的 joint")
    parser.add_argument("--verify-amplitude", type=float, default=0.25, help="关节验证模式的关节偏移幅度")
    return parser.parse_args()


def main() -> None:
    """主入口。"""
    args = parse_args()  # 解析命令行。
    cfg = RunnerConfig(
        scene_xml=args.scene_xml,
        theo_config=args.theo_config,
        policy=args.policy,
        sim_duration=args.duration,
        headless_duration=args.headless_duration,
        stand_time=args.stand_time,
        stabilize_time=args.stabilize_time,
        policy_dt=args.policy_dt,
        kp=args.kp,
        kd=args.kd,
        stand_kp=args.stand_kp,
        stand_kd=args.stand_kd,
        damping_kd=args.damping_kd,
        damping_hold_kp_initial=args.damping_hold_kp_initial,
        damping_hold_decay_time=args.damping_hold_decay_time,
        action_clip=args.action_clip,
        foot_force_scale=args.foot_force_scale,
        foot_force_clip=args.foot_force_clip,
        foot_force_binary=args.foot_force_binary,
        foot_contact_threshold=args.foot_contact_threshold,
        enable_keyboard_cmd=args.enable_keyboard_cmd,
        cmd_step=args.cmd_step,
        cmd_vx=args.cmd_vx,
        cmd_vy=args.cmd_vy,
        cmd_yaw=args.cmd_yaw,
        scripted_test=args.scripted_test,
        turn_yaw=args.turn_yaw,
        turn_duration=args.turn_duration,
        post_turn_stop_duration=args.post_turn_stop_duration,
        forward_vx=args.forward_vx,
        forward_duration=args.forward_duration,
        strafe_vy=args.strafe_vy,
        left_duration=args.left_duration,
        right_duration=args.right_duration,
        verify_joints=args.verify_joints,
        verify_joint=args.verify_joint,
        verify_amplitude=args.verify_amplitude,
    )  # 命令行参数转配置对象。

    print("=" * 78)
    print("Go2 TheoBounac RL Policy -> MuJoCo Sim2Sim Runner")
    print("=" * 78)
    print(f"[MuJoCo] {cfg.scene_xml}")
    print(f"[Theo config] {cfg.theo_config}")

    runner = Go2MujocoRlPolicyRunner(cfg)  # 创建 runner。
    print(f"[Policy] {runner.theo.policy_path}")
    print(f"[Timing] sim_dt={runner.dt:.4f}s, policy_dt={cfg.policy_dt:.4f}s, decimation={runner.policy_decimation}")
    print(
        f"[PD] stand=({cfg.stand_kp}, {cfg.stand_kd}), rl=({cfg.kp}, {cfg.kd}), "
        f"damping_kd={cfg.damping_kd}, damping_hold_kp_initial={cfg.damping_hold_kp_initial}"
    )
    print(f"[Action] scale={runner.theo.action_scale}, clip={cfg.action_clip}")
    print(
        f"[Foot force] scale={cfg.foot_force_scale}, clip={cfg.foot_force_clip}, "
        f"binary={cfg.foot_force_binary}, threshold={cfg.foot_contact_threshold}"
    )
    if cfg.scripted_test != "none":
        print(
            f"[Script] {cfg.scripted_test}: turn_yaw={cfg.turn_yaw}, turn_duration={cfg.turn_duration}, "
            f"stop={cfg.post_turn_stop_duration}, forward_vx={cfg.forward_vx}, forward_duration={cfg.forward_duration}"
        )

    if cfg.verify_joints:
        runner.run_verify_joints()  # 关节映射验证模式。
        return

    if args.headless_test:
        ok = runner.run_headless()  # 无窗口自检。
        raise SystemExit(0 if ok else 1)  # 失败返回非零。

    runner.run_viewer()  # 默认打开 MuJoCo viewer。


if __name__ == "__main__":
    main()
