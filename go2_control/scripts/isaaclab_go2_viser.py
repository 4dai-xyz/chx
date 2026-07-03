#!/usr/bin/env python3
"""用无 Kit 的 Viser 浏览器可视化运行 Isaac Lab Go2 任务。"""

from __future__ import annotations

import argparse  # 解析命令行参数
import contextlib  # 用于安全忽略可选包导入错误
import os  # 读取环境变量
import sys  # 改写 sys.argv 以交给 Isaac Lab preset 解析
from pathlib import Path

import gymnasium as gym  # Isaac Lab 任务通过 Gymnasium registry 创建
import torch  # 创建动作张量，并控制随机种子

import isaaclab.utils.assets as asset_paths  # Isaac Lab 的资产路径常量


def configure_local_asset_root() -> Path:
    """在任务模块导入前，把 Isaac 资产根目录重定向到本地缓存。"""
    root = Path(
        os.environ.get(
            "ISAAC_LOCAL_ASSET_ROOT",
            "/home/ros/isaac_go2/assets_cache/Assets/Isaac/6.0",
        )
    ).resolve()  # 本地 Isaac 资产根目录
    go2_usd = root / "Isaac/IsaacLab/Robots/Unitree/Go2/go2.usd"  # Go2 机器人 USD
    ground_usd = root / "Isaac/Environments/Grid/default_environment.usd"  # 默认地面 USD
    missing = [path for path in (go2_usd, ground_usd) if not path.is_file()]  # 检查关键资产是否存在
    if missing:
        formatted = "\n".join(f"  - {path}" for path in missing)
        raise FileNotFoundError(f"Isaac local assets are incomplete:\n{formatted}")

    asset_paths.NUCLEUS_ASSET_ROOT_DIR = str(root)  # 覆盖 Nucleus 根目录
    asset_paths.NVIDIA_NUCLEUS_DIR = str(root / "NVIDIA")  # 覆盖 NVIDIA 资产目录
    asset_paths.ISAAC_NUCLEUS_DIR = str(root / "Isaac")  # 覆盖 Isaac 资产目录
    asset_paths.ISAACLAB_NUCLEUS_DIR = str(root / "Isaac/IsaacLab")  # 覆盖 IsaacLab 资产目录
    return root  # 返回实际使用的本地资产根目录


LOCAL_ASSET_ROOT = configure_local_asset_root()  # 必须先配置资产路径，再导入任务包

import isaaclab_tasks  # noqa: E402,F401  # 注册 Isaac Lab 官方任务

with contextlib.suppress(ImportError):
    import isaaclab_tasks_experimental  # noqa: E402,F401  # 如果存在实验任务包，也一并注册

from isaaclab_tasks.utils import (  # noqa: E402
    add_launcher_args,  # 添加 --device / --visualizer 等 Isaac Lab 启动参数
    fold_preset_tokens,  # 整理 presets=newton_mjwarp 这类参数
    launch_simulation,  # 根据配置启动对应物理/可视化后端
    resolve_task_config,  # 根据任务名解析 env_cfg
    setup_preset_cli,  # 解析 preset 风格命令行参数
)


parser = argparse.ArgumentParser(
    description="Run Unitree Go2 with Newton/MJWarp and the Viser browser visualizer."
)  # 创建命令行解析器
parser.add_argument("--num_envs", type=int, default=1)  # 并行环境数，浏览器观察默认 1 个
parser.add_argument("--task", type=str, default="Isaac-Velocity-Flat-Unitree-Go2-Play-v0")  # 默认 Go2 Play 任务
add_launcher_args(parser)  # 注入 Isaac Lab 通用启动参数
args_cli, hydra_args = setup_preset_cli(parser)  # 解析普通参数和 Hydra/preset 参数

if not getattr(args_cli, "visualizer", None):
    args_cli.visualizer = ["viser"]  # 没显式指定时默认使用 Viser 浏览器可视化
    args_cli.visualizer_explicit = True  # 告诉 Isaac Lab 这是用户明确选择的可视化后端

if not any(token.startswith(("presets=", "physics=")) for token in hydra_args):
    hydra_args.append("presets=newton_mjwarp")  # 默认使用 Newton/MJWarp，避免 WSL Kit 窗口

sys.argv = [sys.argv[0]] + fold_preset_tokens(hydra_args)  # 让 Isaac Lab 后续解析 preset 参数


def disable_kit_debug_markers(env_cfg) -> None:
    """关闭无 Kit 浏览器模式不需要的 USD 调试 marker。"""
    commands = getattr(env_cfg, "commands", None)  # 任务里的命令管理器配置
    if commands is None:
        return
    for term_name in dir(commands):
        if term_name.startswith("_"):
            continue  # 跳过 Python 内部属性
        term_cfg = getattr(commands, term_name, None)  # 取出每个 command term 配置
        if hasattr(term_cfg, "debug_vis"):
            term_cfg.debug_vis = False  # 关闭速度箭头等 Kit/USD 调试可视化，避免 arrow_x.usd 缺失


def main() -> None:
    torch.manual_seed(42)  # 固定随机种子，让测试更容易复现
    env_cfg, _ = resolve_task_config(args_cli.task, "")  # 根据任务名获取环境配置

    with launch_simulation(env_cfg, args_cli):
        env_cfg.scene.num_envs = args_cli.num_envs  # 设置并行仿真环境数量
        env_cfg.sim.device = args_cli.device  # 设置物理计算设备，例如 cuda:0 或 cpu

        # 独立几何浏览器不需要远程光照/材质资产，关闭后更适合 WSL 本地缓存运行。
        if hasattr(env_cfg.scene, "sky_light"):
            env_cfg.scene.sky_light = None  # 关闭天空光资源
        if getattr(env_cfg.scene, "terrain", None) is not None:
            env_cfg.scene.terrain.visual_material = None  # 关闭地面材质资源
        if getattr(env_cfg.observations, "policy", None) is not None:
            env_cfg.observations.policy.enable_corruption = False  # 关闭训练时观测噪声，观察更稳定
        if getattr(env_cfg, "events", None) is not None:
            if hasattr(env_cfg.events, "base_external_force_torque"):
                env_cfg.events.base_external_force_torque = None  # 关闭随机外力
            if hasattr(env_cfg.events, "push_robot"):
                env_cfg.events.push_robot = None  # 关闭随机推机器人事件
        disable_kit_debug_markers(env_cfg)  # 关闭 Kit 调试 marker
        if os.environ.get("DISABLE_CONTACT_SENSORS", "1") == "1":
            if hasattr(env_cfg.scene, "contact_forces"):
                env_cfg.scene.contact_forces = None  # 关闭接触传感器，降低无 Kit 模式的资源要求
            if hasattr(env_cfg.rewards, "feet_air_time"):
                env_cfg.rewards.feet_air_time = None  # 关闭依赖接触信息的奖励项
            if hasattr(env_cfg.terminations, "base_contact"):
                env_cfg.terminations.base_contact = None  # 关闭依赖接触信息的终止项

        print(f"[INFO] Local Isaac asset root: {LOCAL_ASSET_ROOT}")
        print("[INFO] Visualizer: Viser (kit-less browser view)")
        env = gym.make(args_cli.task, cfg=env_cfg)  # 创建 Isaac Lab Gym 环境
        env.reset()  # 重置环境，初始化机器人状态

        print(f"[INFO] Observation space: {env.observation_space}")
        print(f"[INFO] Action space: {env.action_space}")
        print("[INFO] Action mode: zero joint-position offsets; this is a visual smoke test, not a trained gait policy.")
        print("[INFO] Keep this terminal running and open the Viser URL printed above.")

        sim = env.unwrapped.sim  # 取出底层仿真对象，用于检查可视化器状态
        actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)  # 12 维零关节动作，不是行走策略
        while True:
            if sim.visualizers and not any(
                visualizer.is_running() and not visualizer.is_closed
                for visualizer in sim.visualizers
            ):
                break  # 浏览器可视化器关闭后退出循环
            with torch.inference_mode():
                env.step(actions)  # 推进一步仿真，持续发送零动作

        env.close()  # 释放仿真和可视化资源


if __name__ == "__main__":
    main()  # 脚本入口
