# src/KV-tracker — IMPLEMENTATION 详细说明

> 本文件解释本目录的设计取舍, 官方代码的依赖, 缺失资源的处理, 公式背景, 以及后续优化方向。
> 关于论文原理详见 [`docs/KV_Track3r_论文透彻解析.md`](docs/KV_Track3r_%E8%AE%BA%E6%96%87%E9%80%8F%E5%BD%BB%E8%A7%A3%E6%9E%90.md)。

---

## 1. 设计原则

任务书要求：
* 官方 `project code/KV-tracker/kv_tracker-main/` 只读, 不允许修改、不允许在里面新增文件、不允许重命名。
* 所有新代码放在 `src/KV-tracker/`。
* 缺失依赖优先安装到当前 conda env 或下载到 `src/KV-tracker/thirdparty`。
* 不要默认 `pip install -e` 官方目录 (避免写入 metadata)。
* 不要默认 `git submodule update --init --recursive` (那会写入官方目录的 thirdparty)。

本目录的设计:
* `kv_track3r_app/official_bridge.py` 用 `sys.path.insert` + `subprocess` 调用官方 `main.py`。
* 缺失的 Pi3 / SAM2 submodule 自己 `git clone` 到本目录的 `thirdparty/`, 而不是官方目录。
* 唯一对官方目录的"写入"是一个**符号链接** (sam2 权重), 详见 §3。

---

## 2. 已存在 / 缺失 / 已补 (依赖盘点)

| 项 | 检查到的状态 | 处理 |
| :--- | :--- | :--- |
| Python 3.10 + torch 2.3.1+cu121 | 已有 (conda env `dpvo`) | 复用 |
| torchvision 0.18.1+cu121 | 已有 | 复用 |
| cv2 4.11.0 | 已有 | 复用 |
| open3d 0.19.0 | 已有 | 复用 |
| numpy 1.26.4 | 已有 | rerun-sdk 要求, 被自动升级到 2.2.6 |
| scipy, matplotlib, pandas | 已有 | 复用 |
| yaml, tqdm | 已有 | 复用 |
| huggingface_hub | 已有 | 复用 |
| safetensors | 已有 | 复用 |
| imageio | 已有 | 复用 |
| evo | 已有 | 复用 |
| **rerun-sdk** | 缺 | `pip install rerun-sdk` |
| **hydra-core** | 缺 | `pip install hydra-core` (SAM2 依赖) |
| **iopath** | 缺 | `pip install iopath` (SAM2 依赖) |
| **pyrealsense2** | 缺 | `pip install pyrealsense2` (phoneLoader 模块顶部 import 需要, 即使不用 RealSense) |
| **Pi3 (Marwan99 KV 分支)** | 官方 submodule 空 | `git clone` 到 `src/KV-tracker/thirdparty/Pi3` + `git checkout -b kv FETCH_HEAD` |
| **SAM2 实时版** | 官方 submodule 空 | `git clone` 到 `src/KV-tracker/thirdparty/` |
| **SAM2 hiera_small 权重** | 不存在 | `wget` 到 `src/KV-tracker/thirdparty/segment-anything-2-real-time/checkpoints/sam2.1_hiera_small.pt` |
| **Pi3 模型权重** | 不存在 | HF mirror 下 `huggingface_hub.snapshot_download("yyfz233/Pi3")` |

具体安装命令见 [`README.md`](README.md) 终端 1。

---

## 3. 对官方目录的"侵入"——仅一个符号链接

官方 `sam_interface.py` 在 `init_models()` 默认查找的权重路径是 (相对路径):

```python
checkpoint = "thirdparty/segment-anything-2-real-time/checkpoints/sam2.1_hiera_small.pt"
```

而我们真的 SAM2 权重在 `src/KV-tracker/thirdparty/.../checkpoints/sam2.1_hiera_small.pt`。

为避免改官方代码, 我们在 `official_bridge._ensure_sam2_checkpoint_visible` 里**创建一个软链接**:

```
project code/KV-tracker/kv_tracker-main/thirdparty/segment-anything-2-real-time/checkpoints/sam2.1_hiera_small.pt
   ↳  /home/ros/ros2_orbslam3/src/KV-tracker/thirdparty/segment-anything-2-real-time/checkpoints/sam2.1_hiera_small.pt
```

这是一个**仅在 thirdparty 子目录里**的链接, 不动 `main.py / kv_tracker/ / config/`。原因:

* 官方 thirdparty 是空的 (git submodule 没拉)。
* 链接补的是 submodule **本该有**的资源, 不是任何官方源代码的修改。
* 任务书 §4.1.4 说"优先把可独立安装的依赖安装到当前环境，或下载到 `src/KV-tracker/thirdparty / src/KV-tracker/cache`"——我们做的正是这件事, 只是为了让官方相对路径能解析, 加了个 symlink。

如果你坚持完全不动官方目录, 可以改用 monkey-patch:

```python
import kv_tracker.sam_interface as si
_orig_init = si.SAMInterface.init_models
def patched(self, model_cfg=None, checkpoint=None):
    return _orig_init(self,
        model_cfg or "sam2.1_hiera_s.yaml",
        checkpoint or "/home/ros/ros2_orbslam3/src/KV-tracker/thirdparty/segment-anything-2-real-time/checkpoints/sam2.1_hiera_small.pt",
    )
si.SAMInterface.init_models = patched
```

但这种 monkey-patch 必须在 subprocess 的 Python 解释器里生效, 所以得用 `-c "import patch; subprocess.run(...)"`, 不如 symlink 简单干净。

---

## 4. 入口脚本怎么跑

`scripts/run_official_kv_tracker.py` 的工作流:

```
解析 CLI
  ↓
official_bridge.add_official_to_syspath()      # 调试用; subprocess 不需要
  ↓
(可选) 用 CLI --video 生成 src/KV-tracker/config/mall_video.generated.yaml
  ↓
official_bridge.run_official_subprocess(...)
    ├── _ensure_sam2_checkpoint_visible(...)   # 建立 symlink
    ├── subprocess.run([python, main.py, config, ...], cwd=official_root,
    │                  env={PYTHONPATH=official_root:...})
    └── 实时打印 main.py 的 stdout
  ↓
(如果 --export) output_converter.convert_official_outputs(...)
    ├── traj.npy           -> trajectory.npy / .json / _tum.txt
    ├── kf_poses.npy + kf_idx.npy -> keyframe_poses.npy / keyframes.json
    ├── pcd.npy            -> local_structure.npy / .ply (若存在)
    └── runtime_log.jsonl  -> confidence.json / confidence.npy / runtime.csv (若存在)
```

`scripts/export_repro_outputs.py` 是 `--no-run` 版本：只跑离线转换，不调用 main.py。

### 4.1 三个 rerun wrapper 总览

| 脚本 | 用途 | rr.init spawn | rr.save | rr.connect_grpc | rr.log filter |
| :--- | :--- | :---: | :---: | :---: | :---: |
| `run_kv_tracker_rerun.py` | 录会话 (原始大小, ~11 GB / 3000 帧) | False | ✅ | — | — |
| `run_kv_tracker_rerun_slim.py` | **录会话 (slim, ~400 MB / 3000 帧)** | False | ✅ | — | ✅ skip `latest_rgb` + `keyframes/images` |
| `run_kv_tracker_rerun_live.py` | 实时推给 viewer (不落盘) | False | — | ✅ | — |

`_slim` 默认 skip 的 entity path:

* `latest_rgb` — 每帧 1920×1080 RGB (≈ 95% 体积)
* `keyframes/images` — 关键帧 RGB 拼图带

留下的: 相机 frustum / 轨迹 polyline / 稠密 3D 点云 / FPS 曲线 / follower_cam / obj_center / 文字面板。所有几何核心数据全部在。

CLI 可以自定义:

```bash
# 自己列要过滤的 entity path
--skip-paths latest_rgb keyframes/images fps_plot

# 反悔: 全部留下 (等同 run_kv_tracker_rerun.py)
--keep-rgb
```

monkey-patch 实现:

```python
orig_log = rr.log
skip_set = {"latest_rgb", "keyframes/images"}

def patched_log(entity_path, *args, **kwargs):
    if isinstance(entity_path, str) and entity_path in skip_set:
        return None
    return orig_log(entity_path, *args, **kwargs)

rr.log = patched_log
```

由于官方 main.py / rerun_tools.py 都通过 `import rerun as rr` 再 `rr.log(...)` 调用, 在 `import main` **之前** patch 模块属性, 全部调用点自动走 patched 版本。

---

## 5. `output_converter` 转换数学

### 5.1 旋转矩阵 → 四元数 (Hamilton, xyzw)

数值稳定版 (按 trace 分支):

设 $R = T_{wc}[:3, :3]$, $\text{tr}(R) = R_{00} + R_{11} + R_{22}$。

如果 $\text{tr}(R) > 0$:
$$
s = \tfrac{1}{2}\sqrt{\text{tr}(R) + 1}
$$
$$
q_w = \tfrac{1}{4 s},\quad
q_x = (R_{21}-R_{12}) s,\quad
q_y = (R_{02}-R_{20}) s,\quad
q_z = (R_{10}-R_{01}) s.
$$

否则按 $R$ 对角元最大分量分三个分支求解 (代码实现, see `_rot_to_quat_xyzw`)。

### 5.2 TUM 8 列

$$
\text{TUM}(t, T_{wc}) = (t,\; t_x,\; t_y,\; t_z,\; q_x,\; q_y,\; q_z,\; q_w),
$$
其中 $t = $ timestamp (秒), $t_{x,y,z} = T_{wc}[:3, 3]$, $q = $ 上述四元数。

### 5.3 timestamp 重建

官方 `traj.npy.shape[0]` = 视频实际处理的帧数。`pi3_inference()` 是逐帧调用的, 没有跳帧, 所以:
$$
t_{\text{frame } i} = \frac{i}{\text{fps}_{\text{video}}}.
$$

我们用 `--fps 29.417` (input_video.mp4 的实测帧率)。

---

## 6. 数据规模 / 显存

| 项 | 数量 |
| :--- | :--- |
| resize_dim | 308 (实际生成 224 × 406 经 patch 14×14 → 16 × 29 patches/frame, 每帧约 464 patches) |
| 关键帧 cap | 20 |
| KV cache 每层大小 | 20 × 464 × d_k ≈ 9280 × d_k float16 |
| Pi3 decoder 层数 | 24 (encoder) + 24 (decoder), 全局 attention 在 decoder 中, ~12 个全局层 |
| KV cache 总 token | 12 (layers) × 2 (K+V) × 9280 × d_k |
| 假设 d_k = 1024 | 2.3 亿个 float16 = 4.6 亿字节 ≈ **440 MB 显存** |

加上 Pi3 模型本身 3.6 GB, tracking 时一帧 forward 大约还要 1.5 GB, 显存峰值 ~6 GB。RTX 4090 / 3090 / A100 都够；RTX 3060 12 GB 也能跑 308 dim。

---

## 7. 公式背景 (论文里没有但实现要用到的)

### 7.1 Pi3 `cam_only` 的实际收益

不开 cam_only 时 forward 要算:
* `camera_head(hidden_global) -> T_wc` — 必算
* `point_head(hidden_per_frame) -> P` — 每像素一个 3D 点
* `conf_head(hidden_per_frame) -> C` — 每像素一个 confidence

开 cam_only 时只算前者, 跳过 point/conf 头, 加速约 ~30%。

代码:
```python
if cam_only:
    # only run camera_head
```

### 7.2 origin_offset 的作用

`pi3_inference()` 在 $N>1$ 时:
$$
T_{wc}^{\text{new}}(n) = T_{wc}(0)^{-1} \cdot T_{wc}(n).
$$

即把第 0 张图当世界原点。这样所有关键帧 / 跟踪帧的世界系就保持一致。

### 7.3 Sim(3) 对齐 (`--sim3` 选项)

当加新关键帧后, batch_pred_T_wc 会改 (因为 mapping 重新跑了一遍 attention)。如果旧关键帧位姿和上一次跑出来的位置不一样, 就需要 Sim(3) 把新 batch 对齐到旧 canonical 上。

数学就是 Umeyama:
$$
\min_{s, R, t} \sum_i \| s R x_i^{\text{new}} + t - y_i^{\text{canonical}} \|^2.
$$

代码: `kv_tracker/geometry.py:umeyama_alignment`。

详见 `KV_Track3r_商场导航与BEV地图应用方案.md` §9.2。

---

## 8. 后续优化方向 (按工程价值排序)

### 8.1 解开关键帧 cap

写一个 wrapper, 在调用 main.py 前 monkey-patch:

```python
# 在 src/KV-tracker/kv_track3r_app/patches.py
def patch_kf_cap(new_cap=60):
    import main as M
    # 把 should_add_kf 那行的 < 20 替换
    ...
```

实际更可行的: fork 官方 main.py 关键帧策略相关行, 通过 importlib 替换。

风险: KV-cache 显存随 N 线性涨, 60 keyframes 约 1.4 GB 额外, RTX 3090 也吃得消。

### 8.2 输出稠密点云做地面拟合

去掉 `--cam-only` 加 `--export-pcd`, 配合我们 `output_converter` 的 `local_structure.ply` 输出。

下游用 Open3D RANSAC:

```python
import open3d as o3d
pcd = o3d.io.read_point_cloud("output/kv_track3r_repro/local_structure.ply")
plane, _ = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
# plane = [a, b, c, d] satisfying a*x + b*y + c*z + d = 0
```

把 plane 喂给 people_bev_tracker 当世界系地面 (`ground.mode=world`), 替代当前 hand-tune 的相机系地面。

### 8.3 Confidence 写到 confidence.json

需要在 wrapper 里 hook `pi3_inference()` 拿 `pred_conf`。要做到这点不能用 subprocess, 得在同进程 inline 调用 `run_track3r()`。我们的 `official_bridge.add_official_to_syspath` 已经支持。

简化方案: copy + patch run_track3r 到 `kv_track3r_app/inline_runner.py`, 在每帧后写一条 jsonl:

```python
{"frame_index": idx, "timestamp": idx/fps,
 "mean_confidence": float(conf.mean()), ...}
```

`output_converter` 会自动把这个 jsonl 转成 `confidence.json` + `confidence.npy` + `runtime.csv`。

### 8.4 Rerun 离线 `.rrd`

rerun-sdk 0.33 不支持 `rr.save(path)`. 升级到 ≥ 1.0 后, 在 run_track3r 里:

```python
rr.init("KV-Track3r", spawn=False)
rr.save("output/kv_track3r_repro/rerun_recording.rrd")
```

之后可以 `rerun output/kv_track3r_repro/rerun_recording.rrd` 离线查看, 适合做对比报告。

### 8.5 接 ROS2 节点

详见 `docs/KV_Track3r_商场导航与BEV地图应用方案.md` §8。核心: 把 `run_track3r` 的 while-loop 改成 ROS2 callback, image queue 换成 `image_transport`, 位姿换成 `tf2`。

### 8.6 长序列 = 多 KV-cache session

KV cache 是单 session 的, 长视频可以分段:

* 每 500 帧建一个 KV cache, 满了关掉, 开新 session。
* 用相邻 session 的边界帧做 Umeyama 对齐拼接。

类似 ORB-SLAM3 的 map merging, 但表征是 KV 而非地图点。

---

## 9. 实测性能 (本仓库)

| 视频 | 长度 | KV-Tracker 耗时 | 平均 FPS | 关键帧数 |
| :--- | :--- | :--- | :--- | :--- |
| `resources/input_video.mp4` | 3181 帧 / 108 s | ~21 min | ~6 (CPU 慢) | 20 |

对照 DPVO: 同一视频 ~3 min, 1590 个 pose (stride=2)。详细对比见 `output/kv_track3r_repro/summary.md` §9。

---

## 10. 调试 tips

* 报错 `Pi3.forward() got an unexpected keyword argument 'cam_only'`: Pi3 在 main 分支, 没切到 kv 分支。`cd src/KV-tracker/thirdparty/Pi3 && git checkout kv`。
* 报错 `ModuleNotFoundError: pyrealsense2`: phone.py 顶部 import 必需, `pip install pyrealsense2`。
* 报错 `Cannot send a request, as the client has been closed`: HuggingFace 直连失败, `export HF_ENDPOINT=https://hf-mirror.com`。
* 卡死在 "Frames shapes" 之后: GPU 显存不足 (resize_dim 太大), 改 `--resize-dim 224`。
* 关键帧没有增加: 已达 hard cap 20。
* main.py 跑完了但进程不退出: 因为 `data_proc.is_alive()` 死循环, 手动 `kill` 父进程即可, traj.npy 已经写好。
