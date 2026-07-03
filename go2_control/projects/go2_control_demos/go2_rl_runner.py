#!/usr/bin/env python3
"""
Go2 Isaac Lab 策略在 MuJoCo 中部署的教学版 runner。

本文件是当前 go2_control_demos 目录里的主线文件：
  1. 从 MuJoCo 读取 Go2 机身、关节状态。
  2. 按 Isaac Lab 训练配置拼出 48 维 policy observation。
  3. 调用 Isaac Lab / rsl_rl 导出的 TorchScript policy。
  4. 把 policy 输出 action 转为目标关节角。
  5. 用 PD 力矩控制器驱动 MuJoCo 里的 Go2。

为什么默认 action_scale 不是训练时的 0.25：
  Isaac Lab 用的是 Isaac/PhysX 的 Go2 USD 模型，当前这里用的是 unitree_mujoco
  的 Go2 XML 模型。两个模型的质量、接触、摩擦、初始姿态、执行器细节不完全一致。
  如果直接用训练时 scale=0.25，当前策略会在接管瞬间输出较大动作，MuJoCo 中很容易
  站起后倒下。学习阶段先用 0.08 安全降档，确认闭环跑通后再逐步调大。

推荐运行：
  cd /home/ros/unitree_dev
  env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py

无窗口自检：
  env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py --headless-test

注意：
  这个文件只用于 MuJoCo 仿真学习。不要把这里的 low-level PD/action 直接接到实机。
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import torch


# ============================================================
# 1. 路径与姿态常量
# ============================================================
PROJECT_ROOT = Path(__file__).resolve().parents[2]  # 工程根目录：/home/ros/unitree_dev
DEMO_DIR = Path(__file__).resolve().parent  # 当前 demo 目录：projects/go2_control_demos
GO2_SCENE_XML = PROJECT_ROOT / "src/unitree_mujoco/unitree_robots/go2/scene.xml"  # Go2 平地场景 XML
DEFAULT_POLICY = DEMO_DIR / "policies" / "go2_flat_2000.pt"  # 当前默认使用的 Isaac Lab 导出策略
CMD_VEL_FILE = Path("/tmp/go2_cmd_vel.json")  # ROS2 cmd_vel 桥接文件路径。


# Isaac Lab 的 Go2 默认关节角，actuator 顺序：FR, FL, RR, RL，每条腿 hip/thigh/calf。
ISAAC_DEFAULT_ACTUATOR = np.array([
    -0.1, 0.8, -1.5,   # FR: 右前腿，hip=-0.1, thigh=0.8, calf=-1.5
    0.1, 0.8, -1.5,    # FL: 左前腿，hip=0.1, thigh=0.8, calf=-1.5
    -0.1, 1.0, -1.5,   # RR: 右后腿，hip=-0.1, thigh=1.0, calf=-1.5
    0.1, 1.0, -1.5,    # RL: 左后腿，hip=0.1, thigh=1.0, calf=-1.5
], dtype=np.float64)


# unitree_mujoco 官方 stand_go2.py 的稳定站立角，actuator 顺序同上。
OFFICIAL_STAND_ACTUATOR = np.array([
    0.00571868, 0.608813, -1.21763,    # FR: hip, thigh, calf
    -0.00571868, 0.608813, -1.21763,   # FL: hip, thigh, calf
    0.00571868, 0.608813, -1.21763,    # RR: hip, thigh, calf
    -0.00571868, 0.608813, -1.21763,   # RL: hip, thigh, calf
], dtype=np.float64)


@dataclass
class RunnerConfig:
    """运行配置，集中放在这里方便你做实验时逐项修改。"""

    policy: Path = DEFAULT_POLICY  # TorchScript policy 路径。
    scene_xml: Path = GO2_SCENE_XML  # MuJoCo 场景 XML 路径。
    default_pose: str = "official"  # 默认姿态：official 更稳，isaac 更贴近训练环境。
    action_scale: float = 0.08  # 安全部署 scale；训练值是 0.25，建议稳定后再慢慢调大。
    action_clip: float = 3.0  # action 限幅，防止 out-of-distribution 时关节目标飞掉。
    kp: float = 50.0  # PD 比例增益；当前 XML 下比 Isaac 的 25.0 更稳。
    kd: float = 3.5  # PD 阻尼；当前 XML 下比 Isaac 的 0.5 更稳。
    decimation: int = 4  # 每 4 个 MuJoCo 物理步调用一次策略，对应 Isaac Lab 训练配置。
    stand_time: float = 2.0  # 从 keyframe home 平滑过渡到默认站立姿态的时间。
    duration: float = 120.0  # viewer 模式最长运行时间。
    headless_duration: float = 8.0  # 无窗口自检时长。
    cmd_step: float = 0.2  # 键盘每按一次改变的速度指令幅度。
    max_cmd_xy: float = 1.0  # Isaac Lab 训练时 vx/vy 命令范围是 [-1, 1]。
    max_cmd_yaw: float = 1.0  # Isaac Lab 训练时 yaw_rate 命令范围是 [-1, 1]。
    cmd_source: str = "keyboard"  # 速度命令来源：keyboard 或 file。
    cmd_file: Path = CMD_VEL_FILE  # file 模式读取的共享文件。


# ============================================================
# 2. MuJoCo 与 Isaac Lab 的关节顺序映射
# ============================================================
def build_joint_mappings(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    """
    构建 actuator 顺序和 qpos 顺序之间的映射。

    MuJoCo qpos 顺序来自 XML body 的深度优先遍历：
      FL, FR, RL, RR

    MuJoCo actuator 顺序来自 go2.xml 的 <actuator> 定义：
      FR, FL, RR, RL

    策略 observation/action 使用 qpos 顺序；PD 写入 d.ctrl 时必须转回 actuator 顺序。
    """
    nu = model.nu  # 执行器数量，Go2 为 12。
    actuator_to_qpos = np.zeros(nu, dtype=np.int32)  # actuator[i] 对应 qpos[7 + j] 的 j。
    qpos_to_actuator = np.zeros(nu, dtype=np.int32)  # qpos[7 + j] 对应 actuator[i] 的 i。

    for actuator_id in range(nu):  # 遍历每个执行器。
        joint_id = int(model.actuator_trnid[actuator_id][0])  # 查出该 actuator 驱动哪个 joint。
        qpos_addr = int(model.jnt_qposadr[joint_id])  # 查出该 joint 在 qpos 中的地址。
        qpos_index = qpos_addr - 7  # 前 7 位是 floating base，所以关节相对索引要减 7。
        actuator_to_qpos[actuator_id] = qpos_index  # 记录 actuator -> qpos。
        qpos_to_actuator[qpos_index] = actuator_id  # 记录 qpos -> actuator。

    return actuator_to_qpos, qpos_to_actuator


# ============================================================
# 3. 观测构建
# ============================================================
def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """四元数乘法，输入输出都是 [w, x, y, z]。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=np.float64)


def quat_rotate_inverse(quat_wxyz: np.ndarray, vec_xyz: np.ndarray) -> np.ndarray:
    """把世界坐标系向量旋转到机身坐标系，用于对齐 Isaac Lab 的 base velocity 观测。"""
    q = np.asarray(quat_wxyz, dtype=np.float64)  # MuJoCo free joint 四元数是 [w, x, y, z]。
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)  # 单位四元数的逆是共轭。
    v_quat = np.array([0.0, vec_xyz[0], vec_xyz[1], vec_xyz[2]], dtype=np.float64)  # 向量转纯四元数。
    return quat_multiply(quat_multiply(q_conj, v_quat), q)[1:4]  # q^-1 * v * q 的向量部分。


def build_observation(
    data: mujoco.MjData,
    default_qpos: np.ndarray,
    last_action: np.ndarray,
    cmd_vel: np.ndarray,
) -> np.ndarray:
    """
    构建 48 维 observation，顺序来自训练日志 params/env.yaml。

    0:3   base_lin_vel          机身坐标系线速度
    3:6   base_ang_vel          机身坐标系角速度
    6:9   projected_gravity     重力在机身坐标系中的方向
    9:12  velocity_commands     速度指令 [vx, vy, yaw_rate]
    12:24 joint_pos_rel         当前关节角 - 默认关节角
    24:36 joint_vel_rel         当前关节速度
    36:48 last_action           上一次 policy 输出
    """
    quat = data.qpos[3:7].copy()  # floating base 姿态四元数。
    qpos = data.qpos[7:19].copy()  # 12 个关节角，qpos 顺序。
    qvel = data.qvel[6:18].copy()  # 12 个关节速度，qpos 顺序。

    obs = np.zeros(48, dtype=np.float32)  # Isaac Lab 当前 Go2 flat policy 的输入维度是 48。
    obs[0:3] = quat_rotate_inverse(quat, data.qvel[0:3]).astype(np.float32)  # base_lin_vel。
    obs[3:6] = quat_rotate_inverse(quat, data.qvel[3:6]).astype(np.float32)  # base_ang_vel。
    obs[6:9] = quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0])).astype(np.float32)  # gravity。
    obs[9:12] = cmd_vel.astype(np.float32)  # 外部给定的速度命令。
    obs[12:24] = (qpos - default_qpos).astype(np.float32)  # 关节相对默认角。
    obs[24:36] = qvel.astype(np.float32)  # 关节速度。
    obs[36:48] = last_action.astype(np.float32)  # 上一次动作。
    return obs


def body_pitch_roll(data: mujoco.MjData) -> tuple[float, float]:
    """从 MuJoCo free joint 四元数提取 pitch/roll，用于跌倒保护。"""
    w, x, y, z = data.qpos[3:7]
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))  # 绕 y 轴俯仰角。
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))  # 绕 x 轴横滚角。
    return float(pitch), float(roll)


def has_fallen(data: mujoco.MjData) -> bool:
    """简单跌倒判断：机身太低或姿态角太大就认为不安全。"""
    pitch, roll = body_pitch_roll(data)
    return bool(data.qpos[2] < 0.16 or abs(pitch) > 0.85 or abs(roll) > 0.85)


# ============================================================
# 4. 键盘速度指令
# ============================================================
class CmdVelState:
    """保存当前速度命令，键盘回调会修改它，主循环会读取它。"""

    def __init__(self, cfg: RunnerConfig):
        self.cfg = cfg  # 需要 cfg 里的限幅和步长。
        self.cmd_vel = np.zeros(3, dtype=np.float32)  # [vx, vy, yaw_rate]。

    def get(self) -> np.ndarray:
        """返回当前速度命令副本，避免外部直接改内部数组。"""
        return self.cmd_vel.copy()

    def stop(self) -> None:
        """急停：速度命令清零。"""
        self.cmd_vel[:] = 0.0
        print("\r[cmd] stop vx=+0.0 vy=+0.0 yaw=+0.0          ", end="", flush=True)

    def add(self, dvx: float, dvy: float, dyaw: float) -> None:
        """增量修改速度命令，并限制在 Isaac Lab 训练命令范围内。"""
        self.cmd_vel[0] = np.clip(self.cmd_vel[0] + dvx, -self.cfg.max_cmd_xy, self.cfg.max_cmd_xy)
        self.cmd_vel[1] = np.clip(self.cmd_vel[1] + dvy, -self.cfg.max_cmd_xy, self.cfg.max_cmd_xy)
        self.cmd_vel[2] = np.clip(self.cmd_vel[2] + dyaw, -self.cfg.max_cmd_yaw, self.cfg.max_cmd_yaw)
        print(
            f"\r[cmd] vx={self.cmd_vel[0]:+.1f} vy={self.cmd_vel[1]:+.1f} "
            f"yaw={self.cmd_vel[2]:+.1f}          ",
            end="",
            flush=True,
        )


def make_key_callback(cmd_state: CmdVelState):
    """创建 MuJoCo viewer 的键盘回调函数。"""
    step = cmd_state.cfg.cmd_step  # 每次按键增加/减少的速度命令。

    def callback(keycode: int) -> None:
        from mujoco.glfw import glfw  # 延迟导入，避免无窗口自检时依赖 GLFW 按键常量。

        if keycode == glfw.KEY_UP:
            cmd_state.add(+step, 0.0, 0.0)  # ↑：增加前进速度。
        elif keycode == glfw.KEY_DOWN:
            cmd_state.add(-step, 0.0, 0.0)  # ↓：增加后退速度。
        elif keycode == glfw.KEY_LEFT:
            cmd_state.add(0.0, +step, 0.0)  # ←：增加左移速度。
        elif keycode == glfw.KEY_RIGHT:
            cmd_state.add(0.0, -step, 0.0)  # →：增加右移速度。
        elif keycode == glfw.KEY_Q:
            cmd_state.add(0.0, 0.0, +step)  # Q：左转。
        elif keycode == glfw.KEY_E:
            cmd_state.add(0.0, 0.0, -step)  # E：右转。
        elif keycode in (glfw.KEY_SPACE, glfw.KEY_R):
            cmd_state.stop()  # 空格/R：急停。

    return callback


def read_cmd_vel_file(cfg: RunnerConfig) -> np.ndarray:
    """从 cmd_vel_bridge.py 写入的共享文件读取速度指令。"""
    try:
        with cfg.cmd_file.open("r") as f:
            data = json.load(f)  # 读取 JSON 格式的速度命令。
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return np.zeros(3, dtype=np.float32)  # 文件不存在或写到一半时保持站立。

    age = time.time() - float(data.get("ts", 0.0))  # 计算命令年龄。
    if age > 1.0:
        return np.zeros(3, dtype=np.float32)  # 超过 1 秒没有新命令就自动停住。

    vx = np.clip(float(data.get("vx", 0.0)), -cfg.max_cmd_xy, cfg.max_cmd_xy)  # 限制前后速度。
    vy = np.clip(float(data.get("vy", 0.0)), -cfg.max_cmd_xy, cfg.max_cmd_xy)  # 限制横向速度。
    yaw = np.clip(float(data.get("yaw_rate", 0.0)), -cfg.max_cmd_yaw, cfg.max_cmd_yaw)  # 限制转向速度。
    return np.array([vx, vy, yaw], dtype=np.float32)  # 返回 [vx, vy, yaw_rate]。


# ============================================================
# 5. Runner 主体
# ============================================================
class Go2RlMujocoRunner:
    """把 Isaac Lab policy 接入 MuJoCo 物理仿真的主控制器。"""

    def __init__(self, cfg: RunnerConfig):
        self.cfg = cfg  # 保存运行配置。
        if not cfg.scene_xml.exists():
            raise FileNotFoundError(f"找不到 MuJoCo 场景文件: {cfg.scene_xml}")
        if not cfg.policy.exists():
            raise FileNotFoundError(f"找不到策略文件: {cfg.policy}")

        self.model = mujoco.MjModel.from_xml_path(str(cfg.scene_xml))  # 加载 Go2 XML 模型。
        self.data = mujoco.MjData(self.model)  # 创建 MuJoCo 数据对象。
        self.dt = float(self.model.opt.timestep)  # MuJoCo 物理步长，当前通常是 0.002s。
        self.actuator_to_qpos, self.qpos_to_actuator = build_joint_mappings(self.model)  # 建立顺序映射。

        self.policy = torch.jit.load(str(cfg.policy), map_location="cpu")  # 加载 TorchScript 策略。
        self.policy.eval()  # 推理模式，关闭训练行为。

        self.default_actuator = self._select_default_pose(cfg.default_pose)  # 选择默认站立姿态。
        self.default_qpos = self.default_actuator[self.actuator_to_qpos].copy()  # 默认姿态转 qpos 顺序。
        self.last_action = np.zeros(12, dtype=np.float32)  # 上一次策略输出。
        self.target_qpos = self.default_qpos.copy()  # 当前目标关节角，qpos 顺序。
        self.max_abs_action = 0.0  # 记录最大动作，帮助判断策略是否饱和。
        self.clip_count = 0  # 记录 action 被限幅的次数。
        self.step_count = 0  # 物理步计数。

    @staticmethod
    def _select_default_pose(name: str) -> np.ndarray:
        """根据参数选择默认站立姿态。"""
        if name == "official":
            return OFFICIAL_STAND_ACTUATOR.copy()  # MuJoCo XML 下更稳，推荐初学默认。
        if name == "isaac":
            return ISAAC_DEFAULT_ACTUATOR.copy()  # 更贴近训练环境，但当前 XML 下可能更不稳。
        raise ValueError("--default-pose 只能是 official 或 isaac")

    def reset(self) -> None:
        """从 MuJoCo XML 的 home keyframe 初始化，避免手写高度导致开局穿地或悬空。"""
        if self.model.nkey > 0:
            self.data.qpos[:] = self.model.key_qpos[0].copy()  # 使用 go2.xml 的 keyframe home。
            self.data.qvel[:] = 0.0  # 初速度清零。
        else:
            self.data.qpos[:7] = [0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0]  # 没有 keyframe 时的兜底。
            self.data.qvel[:] = 0.0  # 初速度清零。
        self.data.ctrl[:] = 0.0  # 初始力矩清零。
        self.last_action[:] = 0.0  # 策略历史动作清零。
        self.target_qpos = self.data.qpos[7:].copy()  # 起步目标先等于当前关节角。
        self.max_abs_action = 0.0  # 重置动作统计。
        self.clip_count = 0  # 重置限幅统计。
        self.step_count = 0  # 重置步计数。

    def compute_policy_target(self, cmd_vel: np.ndarray) -> None:
        """策略推理一步：observation -> action -> target_qpos。"""
        obs = build_observation(self.data, self.default_qpos, self.last_action, cmd_vel)  # 拼 48 维观测。
        obs_tensor = torch.from_numpy(obs).unsqueeze(0).float()  # policy 需要 batch 维度。
        with torch.no_grad():
            action = self.policy(obs_tensor).cpu().numpy().squeeze()  # 推理得到 12 维动作。

        raw_max = float(np.max(np.abs(action)))  # 原始动作最大绝对值。
        clipped = np.clip(action, -self.cfg.action_clip, self.cfg.action_clip)  # 防止动作过大。
        if raw_max > self.cfg.action_clip:
            self.clip_count += 1  # 记录发生过限幅，说明策略可能离开训练分布。
        self.max_abs_action = max(self.max_abs_action, raw_max)  # 记录最大动作。
        self.last_action = clipped.astype(np.float32)  # last_action 使用限幅后的动作，更安全。
        self.target_qpos = self.default_qpos + clipped * self.cfg.action_scale  # action 转目标关节角。

    def standup_target(self, sim_time: float) -> None:
        """起立阶段目标：从 keyframe home 平滑过渡到默认站立姿态。"""
        start_qpos = self.model.key_qpos[0, 7:].copy() if self.model.nkey > 0 else self.data.qpos[7:].copy()
        phase = min(1.0, sim_time / max(self.cfg.stand_time, 1e-6))  # 线性进度 0 到 1。
        smooth = phase * phase * (3.0 - 2.0 * phase)  # smoothstep，比线性插值更柔和。
        self.target_qpos = (1.0 - smooth) * start_qpos + smooth * self.default_qpos  # 起立目标。

    def pd_step(self) -> None:
        """执行一步 PD 力矩控制并推进 MuJoCo 物理。"""
        tau_qpos = (
            self.cfg.kp * (self.target_qpos - self.data.qpos[7:19])
            + self.cfg.kd * (0.0 - self.data.qvel[6:18])
        )  # qpos 顺序下的 PD 力矩。
        self.data.ctrl[:] = tau_qpos[self.qpos_to_actuator]  # 转成 actuator 顺序写入 d.ctrl。
        mujoco.mj_step(self.model, self.data)  # 推进一个物理步。
        self.step_count += 1  # 更新步计数。

    def run_step(self, sim_time: float, cmd_vel: np.ndarray) -> None:
        """根据当前阶段执行一轮控制。"""
        if sim_time < self.cfg.stand_time:
            self.standup_target(sim_time)  # 前几秒只负责站稳，不让 policy 立即接管。
        elif self.step_count % self.cfg.decimation == 0:
            self.compute_policy_target(cmd_vel)  # 每 decimation 步调用一次策略。
        self.pd_step()  # 每个物理步都执行 PD 控制。

    def print_summary(self, sim_time: float, prefix: str = "[完成]") -> None:
        """打印运行统计，帮助判断策略是否稳定。"""
        pitch, roll = body_pitch_roll(self.data)
        print(
            f"\n{prefix} t={sim_time:.2f}s, height={self.data.qpos[2]:.3f}m, "
            f"pitch={pitch:+.2f}, roll={roll:+.2f}, "
            f"max|action|={self.max_abs_action:.2f}, clips={self.clip_count}"
        )

    def run_headless(self, cmd_vel: np.ndarray | None = None) -> bool:
        """无窗口自检，用于快速判断当前参数是否会倒。返回 True 表示通过。"""
        self.reset()  # 初始化仿真。
        cmd = np.zeros(3, dtype=np.float32) if cmd_vel is None else cmd_vel.astype(np.float32)  # 自检命令。
        sim_time = 0.0  # 当前仿真时间。
        total_steps = int(self.cfg.headless_duration / self.dt)  # 自检步数。

        for _ in range(total_steps):
            self.run_step(sim_time, cmd)  # 执行一轮控制。
            sim_time += self.dt  # 更新时间。
            if sim_time > self.cfg.stand_time + 0.5 and has_fallen(self.data):
                self.print_summary(sim_time, prefix="[失败]")  # 打印失败状态。
                return False

        self.print_summary(sim_time, prefix="[通过]")  # 打印通过状态。
        return True

    def run_viewer(self) -> None:
        """打开 MuJoCo viewer，键盘控制速度命令。"""
        self.reset()  # 初始化仿真。
        cmd_state = CmdVelState(self.cfg)  # 创建速度指令状态。
        key_callback = make_key_callback(cmd_state)  # 创建键盘回调。
        sim_time = 0.0  # 当前仿真时间。
        last_print = 0.0  # 上次打印状态的时间。

        if self.cfg.cmd_source == "keyboard":
            print("[键盘] ↑/↓ 前后，←/→ 左右，Q/E 转向，空格/R 急停")
        else:
            print(f"[ROS2] 从 {self.cfg.cmd_file} 读取速度命令，超过 1 秒无新命令自动停住")
        print("[提示] 默认 action_scale=0.08 是安全降档；稳定后可试 --action-scale 0.12")
        print("[安全] 如果机身过低或姿态角过大，程序会自动停止，避免继续抽搐。\n")

        with mujoco.viewer.launch_passive(self.model, self.data, key_callback=key_callback) as viewer:
            while viewer.is_running() and sim_time < self.cfg.duration:
                step_start = time.perf_counter()  # 用于实时限速。
                if self.cfg.cmd_source == "file":
                    cmd = read_cmd_vel_file(self.cfg)  # ROS2 桥接模式从文件读命令。
                else:
                    cmd = cmd_state.get()  # 键盘模式从 viewer 按键读命令。
                self.run_step(sim_time, cmd)  # 执行控制。
                viewer.sync()  # 刷新窗口。
                sim_time += self.dt  # 更新时间。

                if sim_time - last_print > 1.0:
                    last_print = sim_time  # 控制状态打印频率。
                    pitch, roll = body_pitch_roll(self.data)
                    print(
                        f"\r[t={sim_time:5.1f}s h={self.data.qpos[2]:.3f} "
                        f"pitch={pitch:+.2f} roll={roll:+.2f} "
                        f"max|a|={self.max_abs_action:.2f} clips={self.clip_count}]",
                        end="",
                        flush=True,
                    )

                if sim_time > self.cfg.stand_time + 0.5 and has_fallen(self.data):
                    self.print_summary(sim_time, prefix="[安全停止]")
                    break

                elapsed = time.perf_counter() - step_start  # 当前循环耗时。
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)  # 尽量保持实时播放速度。

        self.print_summary(sim_time)


# ============================================================
# 6. 命令行入口
# ============================================================
def parse_args() -> argparse.Namespace:
    """解析命令行参数，方便你做参数实验。"""
    parser = argparse.ArgumentParser(description="Go2 Isaac Lab policy -> MuJoCo runner")
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY, help="TorchScript policy 路径")
    parser.add_argument("--default-pose", choices=["official", "isaac"], default="official", help="默认站立姿态")
    parser.add_argument("--action-scale", type=float, default=0.08, help="部署侧 action scale，训练值是 0.25")
    parser.add_argument("--action-clip", type=float, default=3.0, help="action 限幅")
    parser.add_argument("--kp", type=float, default=50.0, help="PD 比例增益")
    parser.add_argument("--kd", type=float, default=3.5, help="PD 阻尼增益")
    parser.add_argument("--duration", type=float, default=120.0, help="viewer 模式最长运行时间")
    parser.add_argument("--headless-test", action="store_true", help="不打开窗口，只做稳定性自检")
    parser.add_argument("--test-vx", type=float, default=0.0, help="headless 自检时的 vx 命令")
    parser.add_argument("--test-vy", type=float, default=0.0, help="headless 自检时的 vy 命令")
    parser.add_argument("--test-yaw", type=float, default=0.0, help="headless 自检时的 yaw_rate 命令")
    parser.add_argument("--cmd-source", choices=["keyboard", "file"], default="keyboard", help="viewer 模式速度命令来源")
    parser.add_argument("--cmd-file", type=Path, default=CMD_VEL_FILE, help="file 模式读取的 cmd_vel JSON 文件")
    return parser.parse_args()


def main() -> None:
    """程序主入口。"""
    args = parse_args()  # 读取命令行参数。
    cfg = RunnerConfig(
        policy=args.policy,
        default_pose=args.default_pose,
        action_scale=args.action_scale,
        action_clip=args.action_clip,
        kp=args.kp,
        kd=args.kd,
        duration=args.duration,
        cmd_source=args.cmd_source,
        cmd_file=args.cmd_file,
    )  # 把命令行参数转成配置对象。

    print("=" * 72)
    print("Go2 Isaac Lab Policy -> MuJoCo Runner")
    print("=" * 72)
    print(f"[模型] {cfg.scene_xml}")
    print(f"[策略] {cfg.policy}")
    print(f"[姿态] default_pose={cfg.default_pose}")
    print(f"[控制] kp={cfg.kp}, kd={cfg.kd}, action_scale={cfg.action_scale}, clip={cfg.action_clip}")

    runner = Go2RlMujocoRunner(cfg)  # 创建 runner。

    if args.headless_test:
        cmd = np.array([args.test_vx, args.test_vy, args.test_yaw], dtype=np.float32)  # 自检速度命令。
        ok = runner.run_headless(cmd)  # 跑无窗口自检。
        raise SystemExit(0 if ok else 1)  # 自检失败时返回非零状态码。

    runner.run_viewer()  # 默认打开窗口运行。


if __name__ == "__main__":
    main()
