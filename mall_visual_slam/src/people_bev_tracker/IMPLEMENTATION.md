# people_bev_tracker — 实现流程与方法原理

> 行人 BEV (Bird's-Eye-View) 跟踪的离线流水线。
> 输入：商场第一人称视频 + DPVO 相机轨迹。
> 输出：二维平面图视频，同时显示相机走过的轨迹和动态行人当前位置。

本文档说明：

1. [整体数据流](#1-整体数据流)
2. [每个模块的输入输出和算法](#2-每个模块的输入输出和算法)
3. [全部数学物理公式](#3-全部数学物理公式)
4. [关键工程取舍](#4-关键工程取舍)
5. [后续优化方向](#5-后续优化方向)
6. [使用方法（多终端命令）](#6-使用方法多终端命令)

---

## 1. 整体数据流

```
┌──────────────────────┐
│ resources/           │
│   input_video.mp4    │  (1920x1080, 29.42 fps, 3181 帧, 108 s)
└─────────┬────────────┘
          │
          │ ① 跑一次 DPVO (单目稀疏 patch VO)
          ▼
┌──────────────────────┐
│ output/dpvo/         │
│   trajectory_tum.txt │  (TUM 8 列: t tx ty tz qx qy qz qw, stride=2)
└─────────┬────────────┘
          │
          │ ② 离线流水线 offline_pipeline.py
          ▼
┌──────────────────────────────────────────────────────────┐
│  for each src_frame:                                     │
│    ts = src_idx / fps                                    │
│    pose = nearest_pose(poses, ts, tol)         (pose_io) │
│    tracked = YOLO-seg + BoT-SORT(frame)   (person_yolo_) │
│    for person in tracked:                                │
│      foot_uv = compute_footpoint(mask, bbox)  (footpoint)│
│      X_w = ground_intersect(foot_uv, K, pose, ground)    │
│      bev_xy = select_bev_axes(X_w, [x,z])  (ground_proj_)│
│      filt = EMA.update(track_id, bev_xy)  (state_filter) │
│    draw BEV(camera trail + people)         (bev_canvas)  │
│    draw debug overlay on original frame                  │
└─────────┬────────────────────────────────────────────────┘
          ▼
┌──────────────────────────────────────────────┐
│ output/people_bev/                           │
│   bev_tracking.mp4         (带行人轨迹线)    │
│   bev_tracking_clean.mp4   (无行人轨迹)      │
│   debug_overlay.mp4        (原视频+bbox)     │
│   people_tracks.json       (逐帧逐人)        │
│   camera_trajectory.json   (逐帧 T_wc)       │
└──────────────────────────────────────────────┘
```

**①** 这一步是「相机定位」：DPVO 从单目视频估出相机自身在世界系下的位姿
$T_{wc}(t) \in SE(3)$。

**②** 这一步是「行人世界定位」：每一帧 YOLO 给出行人 bbox/mask，取 mask 底部
中位点作为「脚底像素」，用相机内参 $K$ 把脚底像素反投成相机系射线，再和
「地面平面」相交得到行人在世界系的脚底点 $X_w \in \mathbb{R}^3$，最后投到
BEV 平面。

---

## 2. 每个模块的输入输出和算法

### 2.1 [`camera_model.py`](people_bev_tracker/camera_model.py)

**职责**：读 ORB-SLAM3 / KannalaBrandt YAML 标定，提供：

* `load_intrinsics(calib_path, video_size)` → 返回内参矩阵 $K$ (3×3)
* `pixel_to_ray(uv, K)` → 单位归一化的相机系射线

**算法**：

```python
K = [[fx, 0, cx],
     [0, fy, cy],
     [0, 0,  1]]
```

如果标定分辨率 $(W_\mathrm{calib}, H_\mathrm{calib})$ 和视频分辨率
$(W_\mathrm{video}, H_\mathrm{video})$ 不一致，自动等比缩放：

$$
s_x = \frac{W_\mathrm{video}}{W_\mathrm{calib}},\quad s_y = \frac{H_\mathrm{video}}{H_\mathrm{calib}},
$$
$$
K' = \mathrm{diag}(s_x, s_y, 1) \cdot K.
$$

像素 → 相机系射线：见 [§3.1](#31-像素到相机系射线)。

> **第一版限制**：标定文件里的 $k_1,k_2,k_3,p_1,p_2$ 畸变系数本版**没有**真正反畸变，
> 当 Pinhole 处理。商场视频本身畸变不大，可接受；后续可换 `cv2.undistort` 一遍图像
> 再喂 YOLO。

### 2.2 [`pose_io.py`](people_bev_tracker/pose_io.py)

**职责**：读 DPVO 输出的 TUM 8 列轨迹，提供时间戳查询。

DPVO 内部以 stride 为单位计 tick。`demo.py` 写出的 TUM 第一列其实是「DPVO
内部 tick」(0, 1, 2, …)，不是秒。为了把它对回视频帧 timestamp：

$$
t_\mathrm{sec} = \mathrm{tick} \cdot \mathrm{stride} / \mathrm{fps}
$$

四元数 $(q_x, q_y, q_z, q_w)$ → 旋转矩阵：见 [§3.2](#32-四元数旋转矩阵)。

**最近邻匹配**：二分查找比 timestamp 小的最大索引，比较前后两个 pose，取
$|\Delta t|$ 最小且 $\le$ tolerance 的。

如果 DPVO 写出的是 $T_{cw}$（少数情况），加 `pose_is_twc: false`，会自动求逆：
$$T_{wc} = T_{cw}^{-1} = \begin{bmatrix} R^\top & -R^\top t \\ 0 & 1 \end{bmatrix}.$$

### 2.3 [`person_yolo_tracker.py`](people_bev_tracker/person_yolo_tracker.py)

**职责**：包装 Ultralytics 官方 YOLO + 内置 BoT-SORT/ByteTrack。

* 模型：`yolo11n-seg.pt`（首次自动下载到当前工作目录），fallback `yolov8n-seg.pt`。
* 跟踪器：`botsort.yaml`（外观+IoU+motion）/ fallback `bytetrack.yaml`（IoU 优先）。
* 调用：`model.track(persist=True, tracker=..., classes=[0])`，`persist=True` 让
  Ultralytics 跨帧维持 tracker 状态。
* mask 来自 `results[0].masks.data`，是低分辨率浮点 mask，按帧大小最近邻
  resize 后二值化。

输出统一打包成 `TrackedPerson(track_id, bbox_xyxy, score, mask)`。

> **第一版限制**：YOLO-seg 的 mask 在小目标、远距离行人上很粗；后续可换 SAM2
> 用 YOLO bbox 做 prompt 拿到精细 mask（见 [§5](#5-后续优化方向)）。

### 2.4 [`footpoint.py`](people_bev_tracker/footpoint.py)

**职责**：把 mask/bbox 收敛成一个「脚底像素」$(u, v)$。

逻辑（mask 优先 → bbox fallback）：

1. 如果 mask 总像素数 ≥ `min_mask_area_px`：
   1. 找 mask 中 $v_\max = \max\{v: \mathrm{mask}(v,u)=1\}$。
   2. 在 $[v_\max - \alpha \cdot (v_\max - v_\min),\; v_\max]$ 这条窄带里取所有 $u$。
   3. $u_\mathrm{foot} = \mathrm{median}(\{u\})$, $v_\mathrm{foot} = v_\max$。
2. 否则退化：$u_\mathrm{foot} = (x_1 + x_2)/2,\; v_\mathrm{foot} = y_2$。

最后 clamp 到图像范围内。`bottom_percent = α = 0.05`。

中位数比平均数稳一些，对裙摆/拖影类形变更鲁棒。

### 2.5 [`ground_projection.py`](people_bev_tracker/ground_projection.py) — **核心**

支持两套地面参数化：

#### A. 世界系地面 (`mode: "world"`)

地面方程：
$$
\mathbf{n}_w^\top \mathbf{X}_w + d = 0,\quad \|\mathbf{n}_w\|=1.
$$

详细推导见 [§3.3](#33-世界系地面相交)。

#### B. 相机系地面 (`mode: "camera"`，第一版**默认**)

第一人称头戴相机的关键观察：**相机相对地面是刚体**——俯仰角 $\alpha$、相机离地高度
$h$ 全视频近似不变。所以在相机系里写地面比在 DPVO 世界系里写地面稳定得多
（DPVO 世界轴并不和重力对齐，因为它只是把第一帧定为单位矩阵）。

相机系下重力方向（向下）单位向量：
$$
\mathbf{g}_c = (0,\;\cos\alpha,\;\sin\alpha)^\top.
$$

地面方程（相机系）：
$$
\mathbf{g}_c^\top \mathbf{X}_c = h.
$$

详细推导见 [§3.4](#34-相机系地面相交)。

### 2.6 [`state_filter.py`](people_bev_tracker/state_filter.py)

每个 `track_id` 一个 EMA + 速度门限：

$$
\mathbf{p}^{(k)}_\mathrm{filt} = \alpha \cdot \mathbf{p}^{(k)}_\mathrm{new} + (1-\alpha) \cdot \mathbf{p}^{(k-1)}_\mathrm{filt}.
$$

速度门限：如果 $\|\mathbf{p}^{(k)}_\mathrm{new} - \mathbf{p}^{(k-1)}_\mathrm{filt}\|/\Delta t > v_\max$
且这是新 ID 出现以后的第一帧大跳，**拒绝**本次更新（保留旧值，
等下一帧观察）。`v_max = 3.0 m/s` ≈ 行人最大速度。

`max_lost_frames=15`（约 0.5 s）超过这个时长还没出现的 ID 不再 active。

### 2.7 [`bev_canvas.py`](people_bev_tracker/bev_canvas.py)

**世界 → 画布**：

$$
\begin{cases}
\mathrm{px} = \dfrac{W}{2} + \dfrac{x - o_x}{r} \\[6pt]
\mathrm{py} = \dfrac{H}{2} - \dfrac{y - o_y}{r}
\end{cases}
$$

其中 $r$ 是 `resolution_m_per_px`，$(o_x, o_y)$ 是画布中心对应的世界坐标。

Canvas 的 $y$ 像素轴向下，世界轴向上，所以是减号。

新增了开关：

* `draw_camera_trail`：是否画相机历史轨迹。
* `draw_people_trails`：是否画行人轨迹线（关掉就是「干净 BEV」，只剩当前定位点）。

### 2.8 [`offline_pipeline.py`](scripts/offline_pipeline.py) — 主循环

伪代码：

```python
for src_idx, frame_bgr in iterate_frames(cap):
    ts = src_idx / fps
    pose = nearest_pose(poses, ts, tol)            # 没 hit 就 None
    tracked = yolo.step(frame_bgr)                 # YOLO + BoT-SORT

    for person in tracked:
        foot = compute_footpoint(person.mask, person.bbox, (H, W))
        if pose is None:
            continue
        if ground_mode == "camera":
            Xw = intersect_camera_ground(foot, K, pose.T_wc, h, alpha)
        else:
            Xw = intersect_world_ground(foot, K, pose.T_wc, n_w, d)
        if Xw is None: continue
        bev = select_bev_axes(Xw, ["x","z"])
        filt = state_filter.update(person.track_id, ts, src_idx, bev, score)

    bev_img = bev_canvas.draw(camera_trail, current_cam, heading,
                              people_history, active_ids, ...)
    debug_img = annotate(frame_bgr, tracked)
    write videos + json
```

### 2.9 [`render_bev_from_json.py`](scripts/render_bev_from_json.py)

**不重跑 YOLO** 的快速重渲染：只读 `people_tracks.json` 和 `camera_trajectory.json`，
重新画 BEV 视频。改配色、改 BEV 范围、关人轨迹时用这个，比重跑全流水线快 30 倍。

---

## 3. 全部数学物理公式

记号：

* $K$：相机内参矩阵 (3×3, 像素单位)
* $\mathbf{X}_w \in \mathbb{R}^3$：世界坐标点
* $\mathbf{X}_c \in \mathbb{R}^3$：相机坐标点
* $T_{wc} = \begin{bmatrix} R_{wc} & \mathbf{C}_w \\ 0 & 1 \end{bmatrix}$：相机到世界 $SE(3)$ 变换
* $\mathbf{C}_w = T_{wc}[:3, 3]$：相机光心在世界系的位置
* $R_{wc}$：相机到世界的旋转
* $\mathbf{r}_c, \mathbf{r}_w$：相机系/世界系下射线方向
* 相机坐标轴约定：**+X 右，+Y 下（图像 y 向下），+Z 前**

### 3.1 像素到相机系射线

像素 $(u, v)$ 反投影：
$$
\tilde{\mathbf{r}}_c = K^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}, \qquad
\mathbf{r}_c = \frac{\tilde{\mathbf{r}}_c}{\|\tilde{\mathbf{r}}_c\|}.
$$

展开：
$$
\tilde{\mathbf{r}}_c = \left(\frac{u - c_x}{f_x},\; \frac{v - c_y}{f_y},\; 1\right)^\top.
$$

物理意义：相机光心在 $\mathbf{0}$，射线方向是 $\mathbf{r}_c$，
3D 空间里像素 $(u,v)$ 对应的射线参数化为 $\mathbf{X}_c = \lambda \mathbf{r}_c, \lambda > 0$。

### 3.2 四元数 → 旋转矩阵

TUM 格式四元数顺序 $(q_x, q_y, q_z, q_w)$。Hamilton 约定下：
$$
R = \begin{bmatrix}
1 - 2(q_y^2 + q_z^2) & 2(q_x q_y - q_w q_z) & 2(q_x q_z + q_w q_y) \\
2(q_x q_y + q_w q_z) & 1 - 2(q_x^2 + q_z^2) & 2(q_y q_z - q_w q_x) \\
2(q_x q_z - q_w q_y) & 2(q_y q_z + q_w q_x) & 1 - 2(q_x^2 + q_y^2)
\end{bmatrix}.
$$

使用前先归一化 $\|q\| = 1$。

### 3.3 世界系地面相交

地面：$\mathbf{n}_w^\top \mathbf{X}_w + d = 0$.

相机系射线变换到世界系：
$$
\mathbf{r}_w = R_{wc} \mathbf{r}_c,\qquad \mathbf{X}_w(\lambda) = \mathbf{C}_w + \lambda \mathbf{r}_w.
$$

代入地面方程：
$$
\mathbf{n}_w^\top (\mathbf{C}_w + \lambda \mathbf{r}_w) + d = 0
\;\Longrightarrow\;
\boxed{\;\lambda = -\,\dfrac{\mathbf{n}_w^\top \mathbf{C}_w + d}{\mathbf{n}_w^\top \mathbf{r}_w}\;}
$$

合法性检查：

* $|\mathbf{n}_w^\top \mathbf{r}_w| > \varepsilon$（射线不平行地面）
* $\lambda > 0$（前向相交，不是反向延长线打到地面）
* $\lambda \le \lambda_\max$（避免数值近无穷）

### 3.4 相机系地面相交

设相机俯仰角 $\alpha$（机头向下为正）。重力方向在相机系：

$$
\mathbf{g}_c = R_x(\alpha) \cdot (0, 1, 0)^\top = (0, \cos\alpha, \sin\alpha)^\top
$$

（推导：相机不俯仰时世界 +Y 朝下，所以重力在相机系是 $(0,1,0)$。相机绕 +X 轴向下俯
仰 $\alpha$ 后，重力相对相机轴相当于绕 +X 反向旋转 $\alpha$ —— 但因为重力本身不变、
是坐标系旋了，所以在新相机系中重力变成 $R_x(\alpha) \cdot (0,1,0)^\top$）。

地面在相机系：所有满足 $\mathbf{g}_c^\top \mathbf{X}_c = h$ 的点，
其中 $h$ 是相机光心离地面在 $\mathbf{g}_c$ 方向上的投影距离（=相机离地高度，
单位与 DPVO 平移单位相同）。

射线代入：
$$
\mathbf{g}_c^\top (\lambda \mathbf{r}_c) = h
\;\Longrightarrow\;
\boxed{\;\lambda = \dfrac{h}{\mathbf{g}_c^\top \mathbf{r}_c}\;}
$$

要求 $\mathbf{g}_c^\top \mathbf{r}_c > 0$，几何含义是「射线方向在重力方向上的分量为正」（射线确实朝地面方向射出）。

求出 $\mathbf{X}_c = \lambda \mathbf{r}_c$ 再变到世界系：
$$
\mathbf{X}_w = R_{wc} \mathbf{X}_c + \mathbf{C}_w.
$$

#### 为什么相机系地面比世界系地面靠谱

* DPVO 是单目 SLAM，第一帧 $T_{wc} = I$，**世界轴 = 第一帧相机轴**。
  如果第一帧相机不水平（用户低头/抬头），世界 $+Y$ 就**不是重力方向**。
* 而「相机离地面高度 $h$」「相机俯仰 $\alpha$」是相对地面的物理量，
  对头戴/胸戴相机近似不变。
* 实测本视频，世界系地面 $y=0$ 时投影成功率只有 14/145 ≈ 10 %；
  换到相机系地面 ($h=0.1, \alpha=15°$) 后成功率 8657/8671 ≈ 99.8 %。

### 3.5 BEV 投影（选轴 + 像素映射）

从 $\mathbf{X}_w = (X, Y, Z)$ 选两轴当 BEV 平面：

| `bev_axes` | $\mathbf{bev}_{xy}$ |
| :--- | :--- |
| `["x", "z"]` (默认) | $(X, Z)$ |
| `["x", "y"]` | $(X, Y)$ |

像素映射：
$$
\mathrm{px} = \frac{W}{2} + \frac{\mathbf{bev}_x - o_x}{r},\qquad
\mathrm{py} = \frac{H}{2} - \frac{\mathbf{bev}_y - o_y}{r}.
$$

$r$ 是 `resolution_m_per_px`（注意单目 DPVO 没有真实米制，这里实际上是
「DPVO 单位 / 像素」），$(o_x, o_y)$ 是画布中心对应的世界坐标。

### 3.6 EMA 滤波 + 速度门限

EMA：
$$
\mathbf{p}^{(k)}_\mathrm{filt} = \alpha\, \mathbf{p}^{(k)}_\mathrm{new} + (1-\alpha)\, \mathbf{p}^{(k-1)}_\mathrm{filt},\qquad \alpha \in (0, 1).
$$

$\alpha$ 大→响应快、噪声大；$\alpha$ 小→平滑、滞后大。这里取 $\alpha=0.35$。

速度门限：
$$
v = \frac{\|\mathbf{p}_\mathrm{new} - \mathbf{p}_\mathrm{filt}^{(k-1)}\|}{\Delta t}.
$$
若 $v > v_\max$ 且 `lost_count = 0`，本次更新拒绝，等下一帧再说。
`v_max = 3.0 m/s`（行人最大速度上限）。

### 3.7 相机朝向投影 (BEV 上的小箭头)

相机在自身坐标系里的「前向」是 $(0, 0, 1)^\top$（约定 +Z 朝前）。
在世界系里：
$$
\mathbf{f}_w = R_{wc} \cdot (0,0,1)^\top
$$
然后按 `bev_axes` 选两个分量画箭头。

---

## 4. 关键工程取舍

| 取舍 | 选择 | 理由 |
| :--- | :--- | :--- |
| 相机定位 | DPVO（不是 ORB-SLAM3） | 这个仓库已经跑通 DPVO；ORB 在低纹理走廊容易丢 |
| 行人检测 | YOLO11n-seg（不是 RT-DETR） | Ultralytics 自带 BoT-SORT，集成最省事 |
| 跟踪 | BoT-SORT 默认 | 比 ByteTrack 多一个 ReID 头；商场遮挡多 |
| 脚底点 | mask 底部中位 u + 最大 v | 比 bbox 底中点对 fragmented mask 更稳 |
| 地面 | **相机系地面**（不是世界系） | DPVO 世界轴和重力不对齐，相机刚体假设更靠谱 |
| 单目尺度 | 不校正 | 第一版没有 GT，只展示几何关系；BEV 标尺不是真米 |
| 平滑 | EMA + 速度门限 | KF 太重；EMA 已经够干净 |
| 数据持久化 | JSON | 方便事后任意角度重渲染（见 `render_bev_from_json.py`）|
| 重渲染 | JSON-only | 改色/改范围/关行人轨迹时，比重跑全流水线快 30× |

---

## 5. 后续优化方向

### 5.1 真实米制尺度

单目 DPVO 不知道真实尺度。可选：

1. **相机外参 + 已知相机高度** $h_\mathrm{real}$（如 1.6 m）：
   * 让 DPVO 在第一帧用相机系地面假设投一次脚底点，得到 DPVO 单位下的距离。
   * 同一脚底点的真实距离可以通过其它先验（VPR 匹配地图、地砖大小、UWB anchors）拿到。
   * 求比例 $s = h_\mathrm{real} / h_\mathrm{dpvo}$，整轨迹平移分量乘 $s$。
2. **IMU 融合**：DPVO 没接 IMU；如果换 DPVO + IMU 版本（DROID-SLAM-VIO 等）可以恢复尺度。
3. **建筑物 / 楼层平面图对齐**：用 ICP 把 DPVO 轨迹和 CAD 地图对齐，自然出尺度。

### 5.2 地面平面在线估计

固定 `camera_height + pitch` 是粗糙近似。可以：

1. 用 DPVO 重建的稀疏点云，跑 RANSAC 平面拟合得到地面方程。
2. 或者每隔 K 帧做一次「地面再校准」：在最近 K 帧的脚底点上 PCA 找最薄方向当法向。
3. 用 SegFormer / Mask2Former 做地面语义分割，把语义地面像素反投后拟合。

### 5.3 更精细的行人 mask

* 接 **SAM2**：用 YOLO 的 bbox 当 prompt，让 SAM2 出精细 mask。
  目前位姿是好的，瓶颈在 mask 边缘抖动。
* 或者直接换 RT-DETRv2 + 一段时序 mask refinement。

### 5.4 行人世界轨迹平滑

EMA 是「逐 ID 一阶低通」，不感知动力学。升级路径：

1. **Constant-Velocity Kalman Filter**：状态 $(x, z, \dot x, \dot z)$，
   预测+量测更新，可以在 occlude 期间做合理外推。
2. **IMM**（多模型）：把 stand / walk / turn 当不同动力学，自动切换。
3. **Social-LSTM / Transformer**：考虑相互影响，做短期预测；对密集人群有帮助。

### 5.5 跨片段 ID 一致性

YOLO 的 BoT-SORT 是在线跟踪，一旦 ID 丢了就重新分配。可以：

1. 用一个独立的 OSNet/CLIP-ReID 当 appearance bank。
2. 短期 occlusion 后做 ReID 拉回原 ID。
3. 当前 195 个 unique ID 中很多是 fragmented，能压到 < 50 个。

### 5.6 把 BEV 变成「真平面图」(map alignment)

* DPVO 出的轨迹是局部坐标系。商场有平面图 / CAD 时，用 ICP/2D 配准把
  DPVO 轨迹「贴到」平面图上。
* 行人位置自动叠到真实平面图上，能给安防/客流分析直接用。

### 5.7 实时 ROS2 节点

把现在的 offline_pipeline 拆成：

* 节点 A：订阅 `/camera/image_raw` 跑 YOLO，发布 `/people/detections`。
* 节点 B：订阅 DPVO `/tf` 和 `/people/detections`，做 ground projection。
* 节点 C：发布 `/people_bev/map_image`（OpenCV BEV）和 `/people_bev/markers`（RViz Marker）。

### 5.8 闭环 / 全局优化

* DPVO 自带 loop closure（`LOOP_CLOSURE=True`），但默认关闭。打开后能压制
  长走廊累积漂移，让 BEV 轨迹自洽。
* 进一步做 GTSAM 因子图：把行人观测当软约束（行人在场景里的固定区域）做联合优化。

### 5.9 多楼层 / 多片段拼接

商场往往跨楼层。可以：

* 用 DPVO 的关键帧 + Place Recognition (NetVLAD / DBoW2) 做跨片段匹配。
* 多段轨迹 ICP 对齐成一张大平面图。

---

## 6. 使用方法（多终端命令）

以下假设：

* 仓库根目录：`/home/ros/ros2_orbslam3`
* Conda 环境：`dpvo`（已装好 `ultralytics` 和本仓库依赖）

> 如果还没装 ultralytics：
> ```bash
> conda activate dpvo
> pip install -e thirdparty/official/ultralytics
> ```

### 6.1 最少终端的流程（**1 个终端**）

只想一口气重跑：

```bash
# 终端 1
cd /home/ros/ros2_orbslam3
conda activate dpvo

# (a) 跑 DPVO，~3 分钟。输出: project code/DPVO/saved_trajectories/input_video_clean.txt
cd "project code/DPVO"
python demo.py \
  --imagedir /home/ros/ros2_orbslam3/resources/input_video.mp4 \
  --calib    calib/custom_mall.txt \
  --name     input_video_clean \
  --stride   2 \
  --save_trajectory

# (b) 把 DPVO 轨迹放到流水线期望的位置
cp saved_trajectories/input_video_clean.txt \
   /home/ros/ros2_orbslam3/output/dpvo/trajectory_tum.txt

# (c) 跑 BEV 离线流水线，~7.5 分钟。输出: output/people_bev/
cd /home/ros/ros2_orbslam3
python src/people_bev_tracker/scripts/offline_pipeline.py \
  --video resources/input_video.mp4 \
  --calib config/KannalaBrandt8_1280x720.yaml \
  --pose  output/dpvo/trajectory_tum.txt \
  --output-dir output/people_bev
```

### 6.2 推荐的 **3 个终端**（边跑边观察）

```bash
# 终端 1: 跑 DPVO 相机轨迹
cd /home/ros/ros2_orbslam3
conda activate dpvo
cd "project code/DPVO"
python demo.py \
  --imagedir /home/ros/ros2_orbslam3/resources/input_video.mp4 \
  --calib    calib/custom_mall.txt \
  --name     input_video_clean \
  --stride   2 \
  --save_trajectory
# 跑完后:
cp saved_trajectories/input_video_clean.txt \
   /home/ros/ros2_orbslam3/output/dpvo/trajectory_tum.txt
```

```bash
# 终端 2: DPVO 跑完后，跑 BEV 离线流水线
cd /home/ros/ros2_orbslam3
conda activate dpvo
python src/people_bev_tracker/scripts/offline_pipeline.py \
  --video resources/input_video.mp4 \
  --calib config/KannalaBrandt8_1280x720.yaml \
  --pose  output/dpvo/trajectory_tum.txt \
  --output-dir output/people_bev
```

```bash
# 终端 3: 监控 GPU 占用 / 看进度
nvidia-smi -l 2
# 或看输出文件大小增长
watch -n 1 'ls -lh output/people_bev/'
```

### 6.3 **4 个终端**：再加一个「干净版重渲染」

`offline_pipeline` 跑完之后，行人 mask + DPVO 投影都已经存进 JSON。
之后改色/改范围/只看「相机轨迹 + 行人点」时，**不要再跑 YOLO**——
用 `render_bev_from_json.py` 直接基于 JSON 重渲染（约 30 秒搞定 3181 帧）：

```bash
# 终端 4: 渲染干净版 BEV（无行人轨迹线），约 30 s
cd /home/ros/ros2_orbslam3
conda activate dpvo
python src/people_bev_tracker/scripts/render_bev_from_json.py \
  --people-json output/people_bev/people_tracks.json \
  --camera-json output/people_bev/camera_trajectory.json \
  --output       output/people_bev/bev_tracking_clean.mp4 \
  --no-people-trails \
  --use-filtered
```

可选参数：

* `--no-camera-trail`：连相机轨迹也不画，只剩 CAM 当前点和行人点。
* `--camera-trail-length 200`：相机只画最近 200 帧的尾巴。
* `--resolution 0.003`：BEV 像素更密。
* `--origin 0.0 0.5`：画布中心对应的世界坐标。
* `--max-lost-frames 5`：超过 5 帧没出现的 ID 不再画当前点（更干净）。

### 6.4 简单的「单帧检测/快速验收」（30 帧）

```bash
# 终端任意一个
cd /home/ros/ros2_orbslam3
conda activate dpvo
python src/people_bev_tracker/scripts/offline_pipeline.py \
  --video resources/input_video.mp4 \
  --calib config/KannalaBrandt8_1280x720.yaml \
  --pose  output/dpvo/trajectory_tum.txt \
  --output-dir output/people_bev_test \
  --max-frames 30
# ~13 秒。验证: 看 output/people_bev_test/debug_overlay.mp4 有没有 bbox + ID
```

### 6.5 查看视频元信息

```bash
python src/people_bev_tracker/scripts/inspect_video.py \
  --video resources/input_video.mp4
```

### 6.6 **5 个终端**：再加 ROS2 (后续)

第一版没有 ROS2 节点。后续如果要：

```bash
# 终端 5: ROS2 RViz 看 BEV marker
source /opt/ros/humble/setup.bash
ros2 launch people_bev_tracker bev_node.launch.py
# (这个 launch 文件还没写，是 §5.7 的后续任务)
```

---

## 7. 输出汇总

`output/people_bev/` 下：

| 文件 | 大小 | 内容 |
| :--- | :--- | :--- |
| `bev_tracking.mp4` | 26 MB | 带行人轨迹线的 BEV |
| `bev_tracking_clean.mp4` | 22 MB | **只有相机轨迹 + 行人当前定位点** |
| `debug_overlay.mp4` | 314 MB | 原视频 + bbox + mask + ID + 世界坐标 |
| `people_tracks.json` | 6.4 MB | 逐帧逐人 (bbox / foot / world_xyz / bev_xy / filtered) |
| `camera_trajectory.json` | 2.1 MB | 逐帧 (frame_index / timestamp / T_wc / bev_xy) |

完整视频运行实测：

* DPVO：**~3 分钟**（GPU）→ 1590 个 pose
* BEV 流水线：**435.9 s ≈ 7.5 分钟**
  * pose hit = 3181 / 3181 (100 %)
  * projection ok = 8657 / 8671 (99.8 %)
  * unique track_id = 206

## 8. 已知限制 / 一句话提醒

1. **没有真米制**：BEV 标的「米」其实是 DPVO 单位。
2. **地面是固定假设**：`camera_height=0.1 DPVO 单位 + pitch=15°`，换视频要重调。
3. **畸变没真反**：KannalaBrandt 当 Pinhole 处理；商场视频差别不大。
4. **mask 较粗**：第一版只用 YOLO-seg，没接 SAM2。
5. **track_id 偏多**（206）：fragmented 较严重，后续需要 ReID 合并。
6. **没有 ROS2 节点**：只是离线 Python 脚本。
