# Go2 策略权重说明

本目录保存 Go2 控制 demo 使用的 TorchScript 策略权重。

请注意：这些权重主要用于 MuJoCo / Isaac Lab / ROS2 控制链路验证。任何策略在真机运行前都必须先经过仿真验证、限速、限力矩和人工安全检查。

## 权重列表

```text
dummy.pt
  零动作占位策略。
  只用于验证 policy 加载、推理接口和控制链路是否跑通。
  不具备行走能力。

go2_flat_199.pt
  本地 Isaac Lab unitree_go2_flat 早期 checkpoint 导出的 TorchScript policy。
  动作较小，适合验证 Isaac Lab policy -> MuJoCo runner 的部署链路。
  不应宣称为成熟可行走策略。

go2_flat_2000.pt
  本地 Isaac Lab unitree_go2_flat 约 2000 iteration checkpoint 导出的 TorchScript policy。
  动作更激进，在当前 MuJoCo XML 中需要降低 action_scale 并做安全检查。
  不建议直接用于真机。

policy_rough.pt
  TheoBounac / Deploy_SimToReal_RL_Go2 路线中的 Go2 预训练策略。
  当前更适合作为 Go2 MuJoCo Sim2Sim 行走策略基线。
  对应 runner:
    ../go2_mujoco_rl_policy_runner.py
```

## 推荐使用顺序

```text
1. dummy.pt
   验证 Python 环境、TorchScript 加载和 action 输出链路。

2. go2_flat_199.pt
   验证本地 Isaac Lab 导出 policy 与 MuJoCo runner 的维度和控制链路。

3. policy_rough.pt
   验证 TheoBounac Go2 52 维预训练策略的 MuJoCo Sim2Sim 站立和低速命令。
```

## policy_rough.pt 快速测试

在 `go2_control/` 根目录下运行：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --policy projects/go2_control_demos/policies/policy_rough.pt \
  --headless-test --headless-duration 30
```

低速命令测试：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --policy projects/go2_control_demos/policies/policy_rough.pt \
  --headless-test --headless-duration 12 --cmd-vx 0.05
```

## 安全要求

```text
1. 不要直接把任何 .pt 策略用于真机。
2. 真机前必须先在 MuJoCo / Isaac Lab 中跑通。
3. 真机前必须限制速度、力矩和 action scale。
4. 真机前必须准备急停。
5. policy_rough.pt 来源于第三方开源路线，请在对外发布前确认许可证和公司合规要求。
```
