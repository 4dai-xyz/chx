#!/usr/bin/env python3
"""
Go2 MuJoCo 物理仿真 PD 控制器 —— 第一步：让 Go2 在物理仿真中站起来。

本文件的作用：
  用 MuJoCo 的物理引擎（mj_step）驱动 Go2，通过 PD 控制器把关节角拉到站立位姿。
  与 mujoco_go2_control_demos.py 的区别：
    - 后者是运动学模式（直接设 qpos + mj_forward，无物理）
    - 本文件是动力学模式（mj_step + PD 力矩，有重力/接触/惯性）
  后续第二步会接入 policy，第三步会接入 ROS2 cmd_vel。

关键参数来源：
  - PD 增益 kp=50, kd=3.5：来自 unitree_mujoco/example/python/stand_go2.py（官方实机值）
  - 站立/趴下关节角：来自 stand_go2.py 的 stand_up_joint_pos / stand_down_joint_pos
  - 关节顺序（actuator ↔ qpos 映射）：通过 MuJoCo API 在运行时自动计算

运行方式：
  cd /home/ros/unitree_dev
  /home/ros/unitree_dev/.venv-unitree/bin/python projects/go2_control_demos/go2_physics_stand.py
"""

from __future__ import annotations

import math
import time
import sys
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


# ============================================================
# 1. 路径配置
# ============================================================
# Go2 平地场景 XML，直接使用 unitree_mujoco 官方文件，无需复制
PROJECT_ROOT = Path(__file__).resolve().parents[2]  # /home/ros/unitree_dev
GO2_SCENE_XML = PROJECT_ROOT / "src/unitree_mujoco/unitree_robots/go2/scene.xml"

# ============================================================
# 2. 关节角定义（来自官方 stand_go2.py）
# ============================================================
# 关节顺序：FR, FL, RR, RL，每条腿 3 个关节（hip, thigh, calf）
# 这个顺序与 go2.xml 中 <actuator> 的定义顺序一致
STAND_UP_JOINT_POS = np.array([
    0.00571868, 0.608813, -1.21763,    # FR: hip, thigh, calf
    -0.00571868, 0.608813, -1.21763,   # FL: hip, thigh, calf
    0.00571868, 0.608813, -1.21763,    # RR: hip, thigh, calf
    -0.00571868, 0.608813, -1.21763,   # RL: hip, thigh, calf
], dtype=np.float64)

STAND_DOWN_JOINT_POS = np.array([
    0.0473455, 1.22187, -2.44375,      # FR: hip, thigh, calf
    -0.0473455, 1.22187, -2.44375,     # FL: hip, thigh, calf
    0.0473455, 1.22187, -2.44375,      # RR: hip, thigh, calf
    -0.0473455, 1.22187, -2.44375,     # RL: hip, thigh, calf
], dtype=np.float64)


# ============================================================
# 3. PD 控制器类
# ============================================================
class Go2PhysicsPDController:
    """
    Go2 物理仿真 PD 控制器。

    负责：
      - 加载 MuJoCo 模型
      - 计算 actuator ↔ qpos 关节映射
      - 每步用 PD 控制力矩驱动关节
      - 平滑的起立/趴下过渡
    """

    def __init__(self, scene_xml: Path):
        # ---- 加载 MuJoCo 模型 ----
        if not scene_xml.exists():
            raise FileNotFoundError(f"找不到场景文件: {scene_xml}")

        self.model = mujoco.MjModel.from_xml_path(str(scene_xml))
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep  # 仿真步长，go2.xml 默认 0.002s

        # ---- 建立 actuator ↔ qpos 关节映射 ----
        # 问题：actuator 定义顺序（FR, FL, RR, RL）与 qpos 关节顺序（FL, FR, RL, RR）不同
        # 解法：用 MuJoCo API 查出每个 actuator 控制哪个 qpos 索引，建立双向映射
        self._build_mapping()

        # ---- PD 控制参数（来自 stand_go2.py 官方值） ----
        self.kp = 50.0   # 比例增益：关节角误差的放大倍数，值越大"弹簧"越硬
        self.kd = 3.5    # 微分增益：抑制关节速度，提供阻尼，防止震荡

        # ---- 当前目标关节角（actuator 顺序） ----
        self.target_q = STAND_DOWN_JOINT_POS.copy()

        print(f"[PD控制器] 模型加载完成: nq={self.model.nq}, nv={self.model.nv}, nu={self.model.nu}")
        print(f"[PD控制器] dt={self.dt:.4f}s, kp={self.kp}, kd={self.kd}")
        print(f"[PD控制器] qpos 关节顺序: FL, FR, RL, RR")
        print(f"[PD控制器] actuator 顺序: FR, FL, RR, RL")

    def _build_mapping(self):
        """
        建立 actuator 索引与 qpos 索引之间的映射。

        背景：
          go2.xml 中 <actuator> 的定义顺序是 FR, FL, RR, RL
          但 MuJoCo 内部 qpos 的关节顺序是 FL, FR, RL, RR（按 <body> 深度优先遍历）
          所以不能简单地用 d.ctrl[i] 对应 d.qpos[7+i]

        解法：
          通过 actuator_trnid 查出每个 actuator 驱动哪个 joint，
          再通过 jnt_qposadr 查出该 joint 在 qpos 中的位置。
        """
        nu = self.model.nu  # 执行器数量 = 12

        # actuator_to_qpos[i] = j: actuator i 的力矩作用在 qpos[7+j] 上
        self.actuator_to_qpos = np.zeros(nu, dtype=np.int32)
        # qpos_to_actuator[j] = i: qpos[7+j] 的力矩由 actuator i 提供
        self.qpos_to_actuator = np.zeros(nu, dtype=np.int32)

        for i in range(nu):
            jid = int(self.model.actuator_trnid[i][0])       # actuator i 驱动的 joint id
            qpos_addr = self.model.jnt_qposadr[jid]           # 该 joint 在 qpos 中的绝对地址
            j = qpos_addr - 7                                  # 转为相对于 qpos[7:] 的索引
            self.actuator_to_qpos[i] = j
            self.qpos_to_actuator[j] = i

    def set_target(self, target_q_actuator_order: np.ndarray):
        """设置目标关节角（actuator 顺序：FR, FL, RR, RL）。"""
        self.target_q = target_q_actuator_order.copy()

    def step(self) -> bool:
        """
        执行一步物理仿真：PD 计算力矩 → 写入 ctrl → mj_step。

        流程：
          1. 把目标关节角从 actuator 顺序转为 qpos 顺序
          2. 按 qpos 顺序计算 PD 力矩：τ = kp*(target - q) + kd*(0 - q̇)
          3. 把力矩从 qpos 顺序转回 actuator 顺序，写入 d.ctrl
          4. 调用 mj_step 推进物理仿真

        返回: 是否需要继续仿真（始终 True）
        """
        # ---- 步骤 1: 目标关节角转为 qpos 顺序 ----
        target_qpos = self.target_q[self.actuator_to_qpos]  # actuator → qpos

        # ---- 步骤 2: PD 控制 ----
        # qpos[7:19]：12 个关节的当前位置（qpos 顺序）
        # qvel[6:18]：12 个关节的当前速度（qpos 顺序）
        current_q = self.data.qpos[7:]     # 当前关节角
        current_dq = self.data.qvel[6:]    # 当前关节角速度

        # PD 公式: τ = kp * (q_desired - q_actual) + kd * (0 - dq_actual)
        # 目标速度始终为 0（我们希望关节停在目标位置）
        tau_qpos = self.kp * (target_qpos - current_q) + self.kd * (0.0 - current_dq)

        # ---- 步骤 3: 力矩转为 actuator 顺序，写入 d.ctrl ----
        self.data.ctrl[:] = tau_qpos[self.qpos_to_actuator]

        # ---- 步骤 4: 推进物理仿真 ----
        mujoco.mj_step(self.model, self.data)

        return True


# ============================================================
# 4. 主程序：起立 → 站立 → 趴下
# ============================================================
def main():
    print("=" * 60)
    print("Go2 物理仿真 PD 控制器 - 站立演示")
    print("=" * 60)

    # ---- 创建控制器 ----
    ctrl = Go2PhysicsPDController(GO2_SCENE_XML)

    # ---- 启动 MuJoCo viewer ----
    with mujoco.viewer.launch_passive(ctrl.model, ctrl.data) as viewer:
        print("[Viewer] MuJoCo 窗口已打开，按 ESC 可提前退出")

        # 仿真总时长
        sim_time = 0.0
        total_duration = 10.0  # 总共仿真 10 秒

        # 起立过渡时长（秒）
        stand_up_duration = 1.2
        # 趴下过渡时长（秒）
        stand_down_duration = 1.2
        # 趴下开始时间
        stand_down_start = total_duration - stand_down_duration

        while viewer.is_running() and sim_time < total_duration:
            step_start = time.perf_counter()

            # ---- 判断当前阶段，计算目标关节角 ----
            if sim_time < stand_up_duration:
                # 阶段 1：趴下 → 站立（平滑过渡）
                # tanh 函数：从 0 平滑过渡到约 1，比线性插值更柔和
                phase = math.tanh(sim_time / stand_up_duration)
                target = (
                    phase * STAND_UP_JOINT_POS
                    + (1.0 - phase) * STAND_DOWN_JOINT_POS
                )
            elif sim_time < stand_down_start:
                # 阶段 2：保持站立（目标不变）
                target = STAND_UP_JOINT_POS.copy()
            else:
                # 阶段 3：站立 → 趴下（平滑过渡）
                elapsed = sim_time - stand_down_start
                phase = math.tanh(elapsed / stand_down_duration)
                target = (
                    phase * STAND_DOWN_JOINT_POS
                    + (1.0 - phase) * STAND_UP_JOINT_POS
                )

            # 设置目标关节角（actuator 顺序）
            ctrl.set_target(target)

            # 执行一步物理仿真
            ctrl.step()

            # 刷新 viewer
            viewer.sync()

            sim_time += ctrl.dt

            # 时间控制：确保仿真步长稳定
            elapsed = time.perf_counter() - step_start
            if elapsed < ctrl.dt:
                time.sleep(ctrl.dt - elapsed)

        print(f"[完成] 仿真结束，总时长 {sim_time:.2f}s")


if __name__ == "__main__":
    main()