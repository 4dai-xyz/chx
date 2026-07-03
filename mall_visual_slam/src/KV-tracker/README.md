# src/KV-tracker — KV-Tracker 复现 + 商场导航接入封装

> 官方 KV-Tracker 在本仓库的**外部封装**。官方代码完全只读，所有新代码都在本目录。
>
> 论文: [KV-Tracker: Real-Time Pose Tracking with Transformers](https://arxiv.org/abs/2512.22581) (CVPR 2026)
> 官方代码: [`project code/KV-tracker/kv_tracker-main/`](../../project%20code/KV-tracker/kv_tracker-main)

---

## 目录结构

```
src/KV-tracker/
├── README.md                                              # 本文件
├── IMPLEMENTATION.md                                      # 详细实现说明
├── config/
│   └── mall_video.yaml                                   # 本仓库视频的 KV-Tracker 配置
├── kv_track3r_app/
│   ├── __init__.py
│   ├── official_bridge.py                                # sys.path / subprocess 调用官方代码
│   ├── export_tools.py                                   # TUM/JSON/PLY/CSV 工具
│   └── output_converter.py                               # traj.npy -> 多种格式
├── scripts/
│   ├── run_official_kv_tracker.py                        # 主入口 (跑官方 + 导出)
│   └── export_repro_outputs.py                           # 仅离线转换
├── docs/
│   ├── KV_Track3r_论文透彻解析.md                         # 论文原理 + 公式
│   └── KV_Track3r_商场导航与BEV地图应用方案.md            # 工程化路线 A/B/C
└── thirdparty/                                            # ⚠️ 必须人工 git clone 进来
    ├── Pi3/                                              # Marwan99/Pi3_w_KV 的 kv 分支
    └── segment-anything-2-real-time/                     # Gy920/SAM2 实时版
```

---

## 快速开始（**4 个终端**）

> 假设仓库根目录: `/home/ros/ros2_orbslam3`，conda 环境: `dpvo`。

### 终端 1：一次性环境补齐 (~20 分钟)

```bash
cd /home/ros/ros2_orbslam3
conda activate dpvo

# 1. 把 Pi3_w_KV 和 SAM2 实时版克隆到本目录的 thirdparty/
mkdir -p src/KV-tracker/thirdparty
git clone https://gh-proxy.com/https://github.com/Marwan99/Pi3_w_KV.git \
  src/KV-tracker/thirdparty/Pi3
cd src/KV-tracker/thirdparty/Pi3
git fetch origin kv && git checkout -b kv FETCH_HEAD     # ← 一定要切到 kv 分支
cd /home/ros/ros2_orbslam3

git clone https://gh-proxy.com/https://github.com/Gy920/segment-anything-2-real-time.git \
  src/KV-tracker/thirdparty/segment-anything-2-real-time

# 2. pip 安装
pip install rerun-sdk hydra-core iopath pyrealsense2
pip install -e src/KV-tracker/thirdparty/Pi3 --no-build-isolation
pip install -e src/KV-tracker/thirdparty/segment-anything-2-real-time --no-build-isolation

# 3. SAM2 权重 (~176 MB)
mkdir -p src/KV-tracker/thirdparty/segment-anything-2-real-time/checkpoints
wget -O src/KV-tracker/thirdparty/segment-anything-2-real-time/checkpoints/sam2.1_hiera_small.pt \
  https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_small.pt

# 4. Pi3 权重 (~3.6 GB)
HF_ENDPOINT=https://hf-mirror.com python -c "
from huggingface_hub import snapshot_download
snapshot_download(repo_id='yyfz233/Pi3')
"
```

### 终端 2：跑 KV-Tracker

```bash
cd /home/ros/ros2_orbslam3
conda activate dpvo
HF_ENDPOINT=https://hf-mirror.com python -u \
  src/KV-tracker/scripts/run_official_kv_tracker.py \
    --official-root "project code/KV-tracker/kv_tracker-main" \
    --config         src/KV-tracker/config/mall_video.yaml \
    --cam-only \
    --resize-dim 308 \
    --export
```

跑完后输出在 `output/kv_track3r_repro/`：

* `traj.npy` `kf_poses.npy` `kf_idx.npy`  ← 官方原始
* `trajectory.npy / .json / _tum.txt`  ← 转换后
* `keyframe_poses.npy` `keyframes.json`
* `confidence.json` `runtime.csv`
* `summary.md`

### 终端 3：监控 GPU

```bash
watch -n 2 nvidia-smi
```

### 终端 4（可选）：Rerun 实时查看

```bash
conda activate dpvo
rerun
# 然后在 终端 2 上加 --rerun 重新启动
```

---

## 配置 `config/mall_video.yaml`

最简：

```yaml
datasource: phoneLoader
que_size: 1
results_path: /home/ros/ros2_orbslam3/output/kv_track3r_repro
scene_dir: /home/ros/ros2_orbslam3/resources/input_video.mp4
```

`scene_dir` 是要跑的视频；`results_path` 是 KV-Tracker `traj.npy` 等落盘位置。

---

## 输出说明

详见 [`output/kv_track3r_repro/summary.md`](../../output/kv_track3r_repro/summary.md)。

| 文件 | 内容 |
| :--- | :--- |
| `trajectory.npy` | (N, 4, 4) float64 — 每帧 $T_{wc}$ |
| `trajectory_tum.txt` | TUM 8 列 (`timestamp tx ty tz qx qy qz qw`) |
| `trajectory.json` | 含 T_wc + translation + quaternion + 时间戳 + mode |
| `keyframe_poses.npy` | (K, 4, 4) — 关键帧 T_wc |
| `keyframes.json` | kf_index + source frame_index + T_wc + quat |
| `local_structure.ply` | （非 cam_only 模式）稠密点云 |
| `confidence.json` | 逐帧置信度统计（cam_only 下为空） |
| `runtime.csv` | 逐帧 pi3_ms / total_ms / fps（第一版未填） |

---

## 已知限制 / 第一版不做的事

1. 官方 main.py 写死关键帧 cap = 20，长视频后段精度下降。
2. `--cam_only` 不输出稠密点云和 confidence；要点云请去掉 `--cam-only` 加 `--export-pcd`。
3. WSL 远程环境 Rerun 视图需要 X 显示，未默认启用；只生成 npy/json 文件。
4. KV-Tracker / Pi3 单目无尺度。如需米制 BEV，按 [`docs/KV_Track3r_商场导航与BEV地图应用方案.md`](docs/KV_Track3r_%E5%95%86%E5%9C%BA%E5%AF%BC%E8%88%AA%E4%B8%8EBEV%E5%9C%B0%E5%9B%BE%E5%BA%94%E7%94%A8%E6%96%B9%E6%A1%88.md) §9 做尺度对齐。
5. 没有 ROS2 节点（离线为主）；topic 设计见同上 §8。
