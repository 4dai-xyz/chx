# Isaac Sim/Lab 云服务器迁移指南

本文说明如何把当前 WSL 中的 Isaac Sim/Lab + Go2 开发内容迁移到原生 Ubuntu GPU 云服务器。目标是在云端使用 Isaac 标准运行方式，不再依赖当前 WSL 中的 Viser 浏览器实验模式。

## 1. 能不能直接运行

结论：不能保证“复制文件夹后 100% 直接运行”，但可以做到“按同版本重建环境后基本可复现运行”。

原因是 Isaac Sim/Lab 依赖的不只是 Python 包，还依赖：

```text
NVIDIA 显卡型号和显存
NVIDIA 驱动版本
Vulkan / RTX 图形能力
Ubuntu 版本
Python / PyTorch / Isaac Sim / Isaac Lab 版本
系统库
资产下载或资产缓存
远程桌面或 Isaac livestream 显示方式
```

所以迁移原则是：

```text
代码、脚本、训练 checkpoint、项目文档可以直接转移。
conda 环境不建议直接转移，应该在云服务器上重建。
IsaacLab 源码可以转移，也可以在云服务器重新克隆同一分支。
Isaac 资产缓存可以转移，但不保证完整；有网络时更建议云端重新预缓存。
```

## 2. 推荐云服务器配置

推荐选择原生 Ubuntu GPU 服务器，而不是 WSL、Docker 半成品镜像或没有图形驱动的纯计算环境。

最低建议：

```text
系统：Ubuntu 22.04 或 Ubuntu 24.04
GPU：NVIDIA RTX 系列或数据中心 RTX/Ada/Ampere GPU
显存：至少 16 GB，训练和复杂场景建议 24 GB 或更多
内存：至少 32 GB，复杂场景建议 64 GB
硬盘：至少 100 GB 空余，建议 200 GB
驱动：云厂商提供的最新稳定 NVIDIA 驱动
网络：能访问 PyPI、GitHub、NVIDIA / Isaac 资产源
```

如果只是跑小规模 Go2 平地任务，显存要求可以低一些；如果要做高真实感室内场景、相机、深度图、RTX LiDAR、多个并行环境，6 GB 或 8 GB 显存会很紧张。

## 3. 云端显示方式

迁到原生 Ubuntu 后，有三种显示方式。

### 3.1 远程桌面方式

如果云服务器提供图形桌面，可以通过 NoMachine、VNC、Parsec、云厂商工作站客户端等方式连接。

这种方式下可以运行 Isaac 标准窗口：

```bash
cd /home/ros/unitree_dev
DISABLE_FABRIC=0 ISAAC_DEVICE=cuda:0 NUM_ENVS=1 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

注意：这条命令打开的是 Isaac/Kit 标准图形程序，不是 Viser。

### 3.2 Headless + Isaac livestream

如果云服务器没有桌面窗口，但支持 Isaac livestream / WebRTC，可以使用 headless 模式启动 Isaac，再用官方客户端或浏览器连接流媒体画面。

这种方式仍然是 Isaac 原生渲染链路，只是显示结果通过流媒体传回来。它不是当前 WSL 中的 Viser 几何可视化。

### 3.3 纯训练方式

如果只是训练强化学习策略，不需要实时画面，可以直接 headless：

```bash
cd /home/ros/unitree_dev
NUM_ENVS=32 MAX_ITERATIONS=50 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_train_small.sh
```

训练完成后再用 play 或 record 观察策略效果。

## 4. 应该转移哪些文件

### 4.1 推荐最简单方案：转移整个 unitree_dev

当前 `/home/ros/unitree_dev` 约 1.3 GB，直接转移最省事：

```text
/home/ros/unitree_dev
```

里面包括：

```text
scripts/isaaclab_install.sh
scripts/isaaclab_check.sh
scripts/isaaclab_go2_play.sh
scripts/isaaclab_go2_train_small.sh
scripts/isaaclab_go2_record.sh
scripts/isaaclab_precache.sh
docs/
README.md
MuJoCo / Unitree SDK / ROS2 相关源码和脚本
```

即使云端暂时只跑 Isaac，也建议保留整个目录，因为后续你可能还要对照 MuJoCo、Unitree SDK、ROS2 和 SLAM。

### 4.2 IsaacLab 源码

当前源码目录：

```text
/home/ros/isaac_go2/IsaacLab
```

这个目录可以转移，也可以在云端重新克隆同一分支。当前安装脚本默认分支是：

```text
release/3.0.0-beta2
```

如果你没有改 IsaacLab 源码，推荐云端重新运行安装脚本，让它自动获取源码。

如果你改过 IsaacLab 源码，必须转移整个 `IsaacLab` 目录，或者把改动提交到自己的 Git 仓库再在云端拉取。

### 4.3 训练日志和 checkpoint

如果你已经训练出了策略，最重要的是转移：

```text
/home/ros/isaac_go2/IsaacLab/logs/rsl_rl/
```

这里通常包含：

```text
训练运行目录
model_*.pt
参数配置
TensorBoard 日志
```

没有 checkpoint 时，`play.py` 只能跑 zero/random agent 或重新训练，不能展示稳定行走策略。

### 4.4 Isaac 资产缓存

当前资产缓存：

```text
/home/ros/isaac_go2/assets_cache
```

这个目录可以转移，但当前缓存不一定完整。之前 `arrow_x.usd` 缺失就说明缓存并不是完整 Isaac 资产库。

推荐策略：

```text
云端有稳定网络 -> 不强制转移 assets_cache，云端重新预缓存。
云端网络很差 -> 可以先转移 assets_cache，再让云端缺什么补什么。
完全离线云端 -> 必须提前准备完整 Isaac 资产缓存，否则不建议。
```

### 4.5 不建议转移 conda 环境

当前本地 conda 环境：

```text
/home/ros/miniconda3/envs/env_isaaclab312
```

大小约 11 GB，不建议直接转移。

原因：

```text
conda 环境包含大量本机路径
与 Python、CUDA、驱动、系统库耦合
复制过去后出错很难排查
云服务器 GPU 驱动和本机 WSL 不一样
```

正确做法是在云端重新创建：

```text
env_isaaclab312
Python 3.12
Isaac Sim / Isaac Lab
PyTorch / rsl_rl
```

## 5. 云端目录建议

最省事的做法是在云服务器也使用同样目录：

```text
/home/ros/unitree_dev
/home/ros/isaac_go2
/home/ros/miniconda3
```

这样当前脚本不用改路径。

如果云端用户名不是 `ros`，比如是 `ubuntu`，也可以使用：

```text
/home/ubuntu/unitree_dev
/home/ubuntu/isaac_go2
/home/ubuntu/miniconda3
```

运行脚本时需要显式指定路径：

```bash
cd /home/ubuntu/unitree_dev
ISAAC_ROOT=/home/ubuntu/isaac_go2 \
CONDA_ROOT=/home/ubuntu/miniconda3 \
CONDA_SH=/home/ubuntu/miniconda3/etc/profile.d/conda.sh \
CONDA_ENV=env_isaaclab312 \
bash scripts/isaaclab_check.sh
```

为了减少初学阶段的路径问题，推荐云端也创建 `ros` 用户或至少保持 `/home/ros` 目录。

## 6. 推荐迁移命令

在本机 WSL 中执行，假设云端地址是 `user@server_ip`，云端也使用 `/home/ros`：

```bash
rsync -avh --progress /home/ros/unitree_dev user@server_ip:/home/ros/
```

如果你已经训练过并有 checkpoint：

```bash
rsync -avh --progress /home/ros/isaac_go2/IsaacLab/logs user@server_ip:/home/ros/isaac_go2/IsaacLab/
```

如果你改过 IsaacLab 源码：

```bash
rsync -avh --progress /home/ros/isaac_go2/IsaacLab user@server_ip:/home/ros/isaac_go2/
```

如果云端网络很差，可以转移资产缓存：

```bash
rsync -avh --progress /home/ros/isaac_go2/assets_cache user@server_ip:/home/ros/isaac_go2/
```

不推荐转移：

```text
/home/ros/miniconda3/envs/env_isaaclab312
```

## 7. 云端安装顺序

### 第 1 步：确认 GPU 和驱动

在云端运行：

```bash
nvidia-smi
```

需要能看到真实 GPU、驱动版本和显存。然后安装基础工具：

```bash
sudo apt update
sudo apt install -y git wget curl build-essential cmake python3-pip vulkan-tools mesa-utils
```

如果运行：

```bash
vulkaninfo
```

能看到 NVIDIA Vulkan 设备，说明图形链路比 WSL 环境健康得多。

### 第 2 步：安装 Miniconda

如果云端还没有 conda，先安装 Miniconda 到：

```text
/home/ros/miniconda3
```

安装完成后确认：

```bash
/home/ros/miniconda3/bin/conda --version
```

### 第 3 步：重建 Isaac Sim/Lab 环境

```bash
cd /home/ros/unitree_dev
bash scripts/isaaclab_install.sh
```

安装完成后检查：

```bash
cd /home/ros/unitree_dev
bash scripts/isaaclab_check.sh
```

在原生 Ubuntu 中，检查结果应该显示：

```text
platform: native Linux
cuda_available: True
isaacsim import ok
Registered Go2 tasks 里能看到 Go2
```

### 第 4 步：预缓存资产

如果有资产预缓存脚本：

```bash
cd /home/ros/unitree_dev
bash scripts/isaaclab_precache.sh
```

如果云端网络好，也可以先不预缓存，第一次运行 Isaac 时让它自动下载。

### 第 5 步：打开 Isaac 标准窗口

远程桌面中运行：

```bash
cd /home/ros/unitree_dev
DISABLE_FABRIC=0 ISAAC_DEVICE=cuda:0 NUM_ENVS=1 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

这条命令使用 `zero_agent.py` 打开场景，只用于确认窗口和模型能显示，不代表已经会稳定行走。

### 第 6 步：训练策略

```bash
cd /home/ros/unitree_dev
NUM_ENVS=32 MAX_ITERATIONS=50 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_train_small.sh
```

训练更久时可以提高 `MAX_ITERATIONS` 和 `NUM_ENVS`，但要根据显存调整。

### 第 7 步：播放 checkpoint

训练完成后，在 `/home/ros/isaac_go2/IsaacLab/logs/rsl_rl/` 里找到运行目录和模型文件，例如：

```text
logs/rsl_rl/unitree_go2_flat/2026-xx-xx_xx-xx-xx/model_50.pt
```

播放：

```bash
cd /home/ros/unitree_dev
LOAD_RUN=<运行目录名> CHECKPOINT=model_50.pt TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

录制：

```bash
cd /home/ros/unitree_dev
LOAD_RUN=<运行目录名> CHECKPOINT=model_50.pt TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_record.sh
```

## 8. 哪些东西可以保证，哪些不能保证

可以保证的是：

```text
当前项目脚本和文档可以迁移。
Go2 Isaac Lab 任务注册逻辑可以迁移。
训练 checkpoint 可以迁移。
MuJoCo / Unitree / ROS2 代码可以随 unitree_dev 一起迁移。
```

不能保证的是：

```text
复制本机 conda 环境后直接运行。
任意云 GPU 都能打开 Isaac 标准窗口。
无桌面云服务器能直接弹出窗口。
不完整 assets_cache 能满足所有 USD 资源。
没有训练 checkpoint 时机器狗能稳定行走。
```

真正的可运行标准是：

```text
nvidia-smi 正常
vulkaninfo 能看到 NVIDIA 设备
bash scripts/isaaclab_check.sh 正常
Isaac 标准窗口或 livestream 能显示
Go2 task 能创建环境
checkpoint 能被 play.py 加载
```

## 9. 建议迁移策略

最稳的流程：

```text
1. 云服务器先不迁移 conda。
2. 转移 /home/ros/unitree_dev。
3. 云端运行 isaaclab_install.sh 重建环境。
4. 云端运行 isaaclab_check.sh。
5. 用 zero_agent 打开窗口确认 Isaac 标准图形链路。
6. 再转移 logs/rsl_rl 下的 checkpoint。
7. 用 play.py 播放策略。
8. 最后再接入 ROS2、ORB-SLAM3、DPVO、VGGT 等导航定位链路。
```

如果只是为了高真实感仿真和录制，云端优先做 Isaac。MuJoCo 可以继续留在本机 WSL，用来做快速控制验证。

