#!/usr/bin/env python3
"""
将 Isaac Lab rsl_rl 训练得到的 Go2 checkpoint 导出为纯 JIT 模型，
去掉 rsl_rl 依赖，只保留 MLP 推理部分，供 MuJoCo runner 加载。

用法:
  PATH=/home/ros/miniconda3/envs/env_isaaclab312/bin:$PATH python export_go2_policy.py
"""
from __future__ import annotations

import os
import sys

CHECKPOINT_PATH = "/home/ros/isaac_go2/IsaacLab/logs/rsl_rl/unitree_go2_flat/2026-06-12_20-15-46/model_1999.pt"
OUTPUT_DIR = "/home/ros/unitree_dev/projects/go2_control_demos/policies"


def main():
    import torch
    import torch.nn as nn

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # 1. 加载 rsl_rl checkpoint
    print(f"[1/4] 加载 checkpoint: {CHECKPOINT_PATH}")
    ckpt = torch.load(CHECKPOINT_PATH, map_location="cpu")

    # 2. 提取 actor MLP 权重
    print("[2/4] 提取 MLP 权重 (48 → 128 → 128 → 128 → 12)")
    sd = ckpt["actor_state_dict"]

    # rsl_rl 的 MLP 结构: Sequential(Linear, ELU, Linear, ELU, Linear, ELU, Linear)
    mlp = nn.Sequential(
        nn.Linear(48, 128),   # mlp.0
        nn.ELU(),             # mlp.1
        nn.Linear(128, 128),  # mlp.2
        nn.ELU(),             # mlp.3
        nn.Linear(128, 128),  # mlp.4
        nn.ELU(),             # mlp.5
        nn.Linear(128, 12),   # mlp.6
    )

    # 加载权重（去掉 "mlp." 前缀）
    model_state = {}
    for key in ["mlp.0.weight", "mlp.0.bias",
                "mlp.2.weight", "mlp.2.bias",
                "mlp.4.weight", "mlp.4.bias",
                "mlp.6.weight", "mlp.6.bias"]:
        model_state[key.replace("mlp.", "")] = sd[key]
    mlp.load_state_dict(model_state)
    mlp.eval()

    # 3. 验证推理
    print("[3/4] 验证推理...")
    obs = torch.zeros(1, 48, dtype=torch.float32)
    with torch.no_grad():
        action = mlp(obs)
    print(f"  输入: {obs.shape} → 输出: {action.shape}")
    print(f"  action range: [{action.min().item():.4f}, {action.max().item():.4f}]")

    # 4. 导出 JIT
    jit_path = os.path.join(OUTPUT_DIR, "go2_flat_2000.pt")
    print(f"[4/4] 导出 JIT: {jit_path}")
    traced = torch.jit.script(mlp)
    traced.save(jit_path)
    print("  导出成功")

    # 验证导出的模型可以加载
    loaded = torch.jit.load(jit_path)
    with torch.no_grad():
        action2 = loaded(obs)
    assert torch.allclose(action, action2), "导出验证失败"
    print("  加载验证通过")

    print(f"\n模型已导出到: {jit_path}")
    print(f"模型大小: {os.path.getsize(jit_path) / 1024:.1f} KB")


if __name__ == "__main__":
    main()