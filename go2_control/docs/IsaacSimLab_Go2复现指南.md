# Isaac Sim/Lab Go2 复现指南

本文说明当前机器上 Isaac Sim / Isaac Lab 的最终保留环境、运行顺序、工作原理和学习路线。

目标：用 Isaac Sim / Isaac Lab 对 Unitree Go2 做 GPU 仿真、强化学习训练和策略播放。

> 平台更正：当前 `/home/ros/isaac_go2` 位于 WSL2。PyTorch CUDA、Python import 和任务注册成功，不代表 RTX/Vulkan 窗口可用。当前 WSL 环境可以保留用于代码阅读、训练前配置检查、实验性无窗口计算，以及 Newton + Viser 浏览器可视化尝试；标准 Isaac 实时窗口、高真实感 RTX 传感器和完整室内场景验证，建议迁移到原生 Windows、原生 Ubuntu 或远程 Linux GPU 机器。RTX 4050 Laptop 6 GB 显存也低于当前 Isaac Sim 官方建议的 16 GB。

## 1. 当前最终结论

现在只保留一套 Isaac Sim / Isaac Lab 环境：

```text
Isaac 工作区：/home/ros/isaac_go2
Isaac Lab 源码：/home/ros/isaac_go2/IsaacLab
Isaac conda 环境：/home/ros/miniconda3/envs/env_isaaclab312
Python：3.12.13
Isaac Sim：6.0.0.0
PyTorch：2.10.0+cu128
Go2 资产缓存：/home/ros/isaac_go2/assets_cache
```

已经删除：

```text
/home/ros/miniconda3/envs/env_isaaclab
/home/ros/isaac_go2/IsaacLab.incomplete.*
/home/ros/isaac_go2/IsaacLab-release-3.0.0-beta2.partial.*
/home/ros/isaac_go2/*.incomplete.*
```

保留但日常不需要管：

```text
/home/ros/isaac_go2/IsaacLab-release_3.0.0-beta2.tar.gz
```

这个压缩包只是源码缓存，不是运行环境。真正运行的是 `/home/ros/isaac_go2/IsaacLab`。

WSL 中如果要尝试实时浏览器画面，使用：

```bash
cd /home/ros/unitree_dev
ISAAC_DEVICE=cuda:0 NUM_ENVS=1 bash scripts/run_isaaclab_go2_viser.sh
```

如果曾经报 `Isaac/Props/UIElements/arrow_x.usd` 缺失，不需要手动补这个文件。那是 Go2 速度命令的调试箭头 marker 触发了额外 USD 资源加载，当前 `scripts/isaaclab_go2_viser.py` 已经在 Viser 模式中自动关闭这些 Kit 调试 marker。

## 2. 和 MuJoCo 是否冲突

不冲突。

MuJoCo 环境：

```text
/home/ros/unitree_dev
/home/ros/unitree_dev/.venv-unitree
/home/ros/unitree_dev/src/unitree_mujoco
/home/ros/unitree_dev/src/unitree_sdk2
/home/ros/unitree_dev/src/unitree_ros2
```

Isaac 环境：

```text
/home/ros/isaac_go2
/home/ros/miniconda3/envs/env_isaaclab312
/home/ros/isaac_go2/assets_cache
```

Isaac 的运行脚本都在 `/home/ros/unitree_dev/scripts/`，但它们只是入口脚本。脚本内部会切到 `/home/ros/isaac_go2/IsaacLab`，并且直接使用 `env_isaaclab312` 的 Python。

脚本运行前会清理：

```text
PYTHONPATH
LD_LIBRARY_PATH
CONDA_PREFIX
CONDA_DEFAULT_ENV
CONDA_SHLVL
CONDA_PROMPT_MODIFIER
```

这样可以避免 DPVO、ROS2、MuJoCo、旧 conda 环境互相污染。

## 3. 不要再使用的命令

不要再使用旧 Isaac 环境：

```bash
conda activate env_isaaclab
```

不要把旧文章里的命令原样照搬：

```bash
cd /home/ros/isaac_go2/IsaacLab
conda activate env_isaaclab
./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py ...
```

原因：

```text
env_isaaclab 已删除
当前环境名是 env_isaaclab312
当前 Isaac Lab 使用 scripts/reinforcement_learning/rsl_rl/*.py 路径
日常运行建议用 /home/ros/unitree_dev/scripts/isaaclab_*.sh 封装脚本
```

## 4. 推荐运行顺序

### 第 1 步：检查 Isaac 环境

```bash
cd /home/ros/unitree_dev
bash scripts/isaaclab_check.sh
```

正常时应看到类似：

```text
Python 3.12.13
torch: 2.10.0+cu128
cuda_available: True
device: NVIDIA GeForce RTX 4050 Laptop GPU
isaacsim import ok
Registered Go2 tasks:
Isaac-Velocity-Flat-Unitree-Go2-Play-v0
Isaac-Velocity-Flat-Unitree-Go2-v0
Isaac-Velocity-Rough-Unitree-Go2-Play-v0
Isaac-Velocity-Rough-Unitree-Go2-v0
```

如果在工具沙盒里看到 `cuda_available: False`，但你自己的终端 `nvidia-smi` 能显示 RTX 4050，则以你自己的 WSL 终端为准。

### 第 2 步：先打开 Go2 场景

先用少量环境数，RTX 4050 Laptop 6GB 显存建议从 `NUM_ENVS=4` 开始：

```bash
cd /home/ros/unitree_dev
NUM_ENVS=4 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

如果你没有训练过模型，这个脚本会自动使用 `zero_agent.py` 打开 Go2 场景。它只用于确认 Isaac 画面、Go2 资产和任务能正常加载，不代表已经加载了会走路的强化学习策略。

当前脚本默认使用：

```text
DISABLE_FABRIC=0
ISAAC_DEVICE=cuda:0
HEADLESS=0
```

也就是启用 Fabric、使用第 0 块 CUDA GPU、打开可视化窗口。之前如果使用 `DISABLE_FABRIC=1`，在 Isaac Lab 3.0 + WSL 的 Go2 任务里可能会出现 `ProxyArray` 类型错误，表现为场景创建到一半就退出，窗口不会弹出来。

如果想显式指定 GPU/Fabric，可以运行：

```bash
cd /home/ros/unitree_dev
DISABLE_FABRIC=0 ISAAC_DEVICE=cuda:0 NUM_ENVS=4 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

如果只是排查 CPU 路径，可以临时测试：

```bash
cd /home/ros/unitree_dev
ISAAC_DEVICE=cpu NUM_ENVS=1 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

CPU 模式只适合排查，不适合训练或正常仿真。

如果想测试动作通道，可以使用随机动作：

```bash
cd /home/ros/unitree_dev
AGENT_MODE=random NUM_ENVS=4 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

随机动作可能让机器人抖动或摔倒，这是正常的。

粗糙地形场景：

```bash
cd /home/ros/unitree_dev
NUM_ENVS=4 TASK=Isaac-Velocity-Rough-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

如果要播放训练好的策略，需要先有 checkpoint。否则 `rsl_rl/play.py` 会去找：

```text
/home/ros/isaac_go2/IsaacLab/logs/rsl_rl/unitree_go2_flat
```

如果这个目录不存在，就说明你还没有训练过本地策略。

### 第 3 步：小规模训练测试

目标不是一次训练出好策略，而是验证训练链路能跑：

```bash
cd /home/ros/unitree_dev
NUM_ENVS=16 MAX_ITERATIONS=50 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_train_small.sh
```

显存不够就降到：

```bash
NUM_ENVS=4 MAX_ITERATIONS=10
```

确认平地任务能跑后，再试粗糙地形：

```bash
cd /home/ros/unitree_dev
NUM_ENVS=16 MAX_ITERATIONS=50 TASK=Isaac-Velocity-Rough-Unitree-Go2-v0 bash scripts/isaaclab_go2_train_small.sh
```

### 第 4 步：查看训练日志

训练日志位于：

```text
/home/ros/isaac_go2/IsaacLab/logs
```

启动 TensorBoard：

```bash
cd /home/ros/isaac_go2/IsaacLab
/home/ros/miniconda3/envs/env_isaaclab312/bin/python -m tensorboard.main --logdir=logs
```

浏览器打开终端提示的地址，重点看：

```text
reward
episode length
value loss
surrogate loss
learning rate
```

### 第 5 步：加载 checkpoint 播放

训练后的模型通常在：

```text
/home/ros/isaac_go2/IsaacLab/logs/rsl_rl/<task>/<run_name>/model_*.pt
```

播放某个 checkpoint：

```bash
cd /home/ros/unitree_dev
LOAD_RUN=<训练文件夹名> CHECKPOINT=<模型文件名> TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_record.sh
```

如果只想播放不录制，可以直接使用 `isaaclab_go2_play.sh`：

```bash
cd /home/ros/unitree_dev
CHECKPOINT=/home/ros/isaac_go2/IsaacLab/logs/rsl_rl/unitree_go2_flat/<训练文件夹名>/<模型文件名> \
TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 \
NUM_ENVS=4 \
bash scripts/isaaclab_go2_play.sh
```

也可以在 IsaacLab 目录手动运行，但建议仍使用脚本思路：

```bash
cd /home/ros/isaac_go2/IsaacLab
PATH=/home/ros/miniconda3/envs/env_isaaclab312/bin:$PATH ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
  --num_envs 4 \
  --load_run <训练文件夹名> \
  --checkpoint <模型文件名>
```

## 5. Isaac 文件架构

```text
/home/ros/isaac_go2
├── IsaacLab/
│   ├── isaaclab.sh                      # Isaac Lab 主入口脚本
│   ├── apps/                            # Isaac/Kit 应用配置
│   ├── scripts/reinforcement_learning/  # rsl_rl、skrl 等训练/播放脚本
│   ├── source/                          # Isaac Lab Python 包和任务源码
│   ├── docs/                            # Isaac Lab 官方文档源码
│   ├── tools/                           # 辅助工具
│   └── logs/                            # 训练日志，运行后生成
├── assets_cache/
│   └── Assets/Isaac/6.0/Isaac/IsaacLab/Robots/Unitree/Go2/
│       ├── go2.usd
│       └── Props/instanceable_meshes.usd
└── IsaacLab-release_3.0.0-beta2.tar.gz  # 源码包缓存
```

Isaac Python 环境：

```text
/home/ros/miniconda3/envs/env_isaaclab312
├── bin/python
├── bin/isaacsim
└── lib/python3.12/site-packages/
    ├── isaacsim*
    ├── torch
    ├── torchvision
    ├── rsl_rl
    ├── gymnasium
    └── isaaclab*
```

工作区封装脚本：

```text
/home/ros/unitree_dev/scripts/isaaclab_check.sh
/home/ros/unitree_dev/scripts/isaaclab_go2_play.sh
/home/ros/unitree_dev/scripts/isaaclab_go2_train_small.sh
/home/ros/unitree_dev/scripts/isaaclab_go2_record.sh
/home/ros/unitree_dev/scripts/isaaclab_precache.sh
/home/ros/unitree_dev/scripts/isaaclab_install.sh
```

## 6. 工作原理

Isaac 这条路线可以理解成四层：

```text
Isaac Sim / PhysX
  负责 GPU 物理仿真、碰撞、接触、关节动力学、渲染和传感器。

Isaac Lab
  负责把机器人任务工程化：场景、机器人、地形、观测、动作、奖励、终止条件、随机化。

RSL-RL
  负责强化学习算法，当前常用 PPO。

Go2 Task
  定义 Unitree Go2 的模型、关节、PD 控制、速度跟踪任务和奖励函数。
```

一次强化学习训练循环大致是：

```text
1. 同时创建 N 个 Go2 仿真环境。
2. 每个环境给策略网络一组观测，例如机身速度、姿态、关节角、关节速度、目标速度。
3. 策略网络输出动作，通常对应 12 个腿部关节的目标位置偏移。
4. Isaac Lab 把动作转换成关节控制目标。
5. PhysX 推进物理仿真，计算接触、碰撞、重力和关节运动。
6. 任务根据速度跟踪、稳定性、能耗、动作平滑程度计算 reward。
7. RSL-RL 用 PPO 更新策略网络。
8. 每隔一段时间保存 checkpoint。
```

Go2 速度跟踪任务的目标：

```text
给定目标 vx、vy、wz
让 Go2 机身速度尽量接近目标
保持机身姿态稳定
减少能耗和动作抖动
在粗糙地形上尽量不摔倒
```

## 7. 和知乎文章命令的对应关系

很多旧教程里的路径类似：

```bash
./isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py
./isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py
```

当前 Isaac Lab 路径是：

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py
```

你复现文章时，先理解文章要做的事情，不要机械照抄路径：

```text
旧文章的目标：Go2 task + RSL-RL PPO + Isaac GPU 并行仿真
当前环境的实现：env_isaaclab312 + Isaac Sim 6.0 + IsaacLab release/3.0.0-beta2
```

## 8. RTX 4050 Laptop 的建议参数

你的显卡是 RTX 4050 Laptop，约 6GB 显存。建议：

```text
可视化播放：NUM_ENVS=1 到 4
首次训练测试：NUM_ENVS=4 到 16，MAX_ITERATIONS=10 到 50
稳定后尝试：NUM_ENVS=32
不要一开始使用：NUM_ENVS=4096
```

如果出现显存不足：

```text
降低 NUM_ENVS
使用 --headless 训练
关闭其他占 GPU 的程序
先跑 Flat，再跑 Rough
```

## 9. 后续学习路线

建议按这个顺序：

```text
1. 先会运行 isaaclab_check.sh，确认 Python、Torch、CUDA、Go2 task。
2. 跑 Isaac-Velocity-Flat-Unitree-Go2-v0 的 play。
3. 跑 Flat 任务小规模训练，学会看 logs 和 TensorBoard。
4. 找到 task 配置文件，理解 observation、action、reward、termination。
5. 修改奖励权重或命令范围，观察训练变化。
6. 跑 Rough 任务，理解地形和 domain randomization。
7. 学习 checkpoint 播放和视频录制。
8. 再研究 sim-to-real：策略导出、观测对齐、控制频率、实机安全限制。
```

## 10. 常见问题

### `ModuleNotFoundError: No module named 'gymnasium'`

通常是运行到了旧环境。解决：

```bash
cd /home/ros/unitree_dev
bash scripts/isaaclab_check.sh
```

不要使用 `conda activate env_isaaclab`。

### conda 出现 `zstandard` 或 `libmamba` 警告

当前脚本已设置：

```text
CONDA_NO_PLUGINS=true
CONDA_SOLVER=classic
```

日常运行 Isaac 不需要手动进入 conda。优先用脚本。

### `nvidia-smi` 能看到显卡，但脚本里 CUDA false

如果是在工具沙盒里运行，可能是沙盒没有 GPU 权限。请在你自己的 WSL 终端运行：

```bash
nvidia-smi
cd /home/ros/unitree_dev
bash scripts/isaaclab_check.sh
```

如果 Isaac 日志出现：

```text
No CUDA context manager available
forcing CPU simulation
Error launching kernel ... expects an array ... but passed value has type ProxyArray
```

先确认你没有手动使用旧参数 `DISABLE_FABRIC=1`。当前 Go2 播放建议使用：

```bash
cd /home/ros/unitree_dev
DISABLE_FABRIC=0 ISAAC_DEVICE=cuda:0 NUM_ENVS=1 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

如果仍然出现 `No CUDA context manager available`，再判断为 GPU/CUDA 上下文没有被 Isaac 正确拿到。请在同一个终端运行：

```bash
nvidia-smi
cd /home/ros/unitree_dev
bash scripts/isaaclab_check.sh
```

`isaaclab_check.sh` 里应看到：

```text
cuda_available: True
device: NVIDIA GeForce RTX 4050 Laptop GPU
```

如果这里是 `False`，不要继续训练，先重启 WSL 或检查 Windows NVIDIA 驱动/WSL GPU。

### Isaac 首次启动很慢

正常。第一次会解压/缓存 Omniverse 扩展和 USD 资产。以后会快一些。缓存主要在：

```text
/home/ros/.local/share/ov
/home/ros/isaac_go2/assets_cache
/home/ros/.cache/pip
```
