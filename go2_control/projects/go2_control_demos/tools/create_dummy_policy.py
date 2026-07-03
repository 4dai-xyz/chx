#!/usr/bin/env python3
"""
创建 dummy policy（零动作策略），用于验证 policy 部署链路是否正常。

Dummy policy 始终输出全零动作：
  target = 0 * action_scale + default_angles = default_angles
  → 机器人保持站立不动

运行方式：
  cd /home/ros/unitree_dev
  /home/ros/unitree_dev/.venv-unitree/bin/python projects/go2_control_demos/tools/create_dummy_policy.py
"""

import sys
from pathlib import Path

import torch
import torch.nn as nn


# ============================================================
# DummyPolicy: 始终输出零动作
# ============================================================
class DummyPolicy(nn.Module):
    """
    零动作策略，用于验证部署链路。

    输入: (batch_size, 47) - 观测向量
    输出: (batch_size, 12) - 动作向量，全零
    """

    def __init__(self, num_obs: int = 47, num_actions: int = 12):
        super().__init__()
        self.num_obs = num_obs        # 观测空间维度
        self.num_actions = num_actions  # 动作空间维度

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        batch_size = obs.shape[0]
        return torch.zeros(batch_size, self.num_actions, device=obs.device)


def main():
    # 输出路径：policies 目录
    output_dir = Path(__file__).resolve().parent.parent / "policies"
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / "dummy.pt"

    print("创建 dummy policy（零动作）...")
    policy = DummyPolicy(num_obs=47, num_actions=12)
    policy.eval()

    # 用随机输入测试一次推理
    test_obs = torch.randn(1, 47)
    with torch.no_grad():
        action = policy(test_obs)
    print(f"  测试推理: 输入(1,47) → 输出{action.shape}: {action.numpy().squeeze()}")

    # 用 TorchScript 导出，兼容 deploy_mujoco.py 的 torch.jit.load()
    traced = torch.jit.script(policy)
    traced.save(str(output_path))
    print(f"  已保存: {output_path}")
    print("完成。")


if __name__ == "__main__":
    main()