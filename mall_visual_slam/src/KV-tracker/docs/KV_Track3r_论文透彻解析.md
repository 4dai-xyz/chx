# KV-Tracker / KV_Track3r 论文透彻解析

> 论文: **KV-Tracker: Real-Time Pose Tracking with Transformers** (CVPR 2026)
> 作者: Marwan Taher, Ignacio Alzugaray, Kirill Mazur, Xin Kong, Andrew J. Davison
> 单位: Dyson Robotics Lab, Imperial College London
> arXiv: https://arxiv.org/abs/2512.22581 ・ Project: https://marwan99.github.io/kv_tracker/

本仓库内对应:

* 论文 PDF: [KV_Track3r.pdf](../../../KV_Track3r.pdf)
* 官方代码: [`project code/KV-tracker/kv_tracker-main/`](../../../project%20code/KV-tracker/kv_tracker-main) (只读)
* 复现封装: [`src/KV-tracker/`](../)

---

## 1. 一句话总结

**KV-Tracker = "把 π³ 的全局 self-attention 在关键帧上跑一遍，把每一层的 Key/Value 缓存下来当场景隐式表示；新来的帧只做一次 cross-attention 去查这堆 KV，从而把多视图几何网络从离线工具变成实时 (≈27 FPS) 6-DoF tracker。"**

---

## 2. 论文试图解决的问题

近期的多视图前馈几何网络 (DUSt3R, MASt3R, VGGT, π³, MapAnything, ...) 在「一次性吃 N 张图，输出全局一致的位姿和点图」上效果非常好，但有两个**致命缺陷**让它们用不上实时系统:

1. **复杂度二次增长**: 全局 self-attention 的 Q/K/V 都是「所有 N 帧的所有 patch token」拼起来算的, 复杂度 $\mathcal{O}((NM)^2)$。每来一帧都重算所有历史帧的 K/V → 不可承受。
2. **不能"关闭循环"**: 如果只用最近窗口 (e.g. VGGT 滑窗) 跑，长序列会丢失对早期场景的记忆, 在物体旋转回到旧视角时会漂。

KV-Tracker 的核心 insight: **关键帧之间的 K/V 一旦算好就不应该改变**, 因为它们代表的是"我对这片场景已经看过的样子"。新来的帧仅以 query 身份去查询这个静态记忆即可。

---

## 3. 背景: 为什么多视图 Transformer 很强但很慢

π³ / VGGT / DUSt3R 这类网络的典型流程:

1. 用 DINOv2 编码每张图得到 token: $\text{Enc}(I_{1:N}) = X_{1:N}$, $X_n \in \mathbb{R}^{M \times d_k}$, 其中 $M$ 是每帧 token 数, $d_k$ 是特征维度。
2. 通过 $L$ 个 decoder block, 在「逐帧 self-attention (frame-wise)」和「全局 self-attention (global)」之间交替。全局 block 才是真正让多帧互相"看见"的地方。
3. 解码头 $\text{decoder heads}(X_{1:N}) = T_{1:N}, P_{1:N}, C_{1:N}$ 给出位姿、点图、置信度。

代码对照 (`src/KV-tracker/thirdparty/Pi3/pi3/models/pi3.py`):

* `self.encoder = Dinov2Model(...)`
* `self.decoder = nn.ModuleList(...)` — 一系列 block, 偶数 idx 是 frame-wise, 奇数 idx 是 global (`store_cache and (i % 2) != 0` 这一行就是认全局块)
* `self.camera_head, self.point_head, self.conf_head` — 三个解码头

代码里的 KV-cache 触发条件:

```python
# pi3/models/pi3.py
def decode(self, hidden, N, H, W, store_cache=False, use_cache=False, tokens_mask=None):
    ...
    for i, block in enumerate(self.decoder):
        if store_cache and (i % 2) != 0:    # 奇数层 = global attention
            # 缓存这一层算出的 K/V
            ...
```

---

## 4. π³ 网络的输入输出

输入:

* 图像 batch $I_{1:N} \in [0, 1]^{N \times 3 \times H \times W}$
* (可选) 内参 $K \in \mathbb{R}^{3 \times 3}$, ray, pose 等多模态先验

输出 (在 `pi3_inference()` 里包成 dict):

* `camera_poses`: $T_{wc} \in \mathbb{R}^{N \times 4 \times 4}$ — 每张图的相机到世界位姿
* `points`: $P \in \mathbb{R}^{N \times H \times W \times 3}$ — 每像素的 3D 点 (在世界系)
* `local_points`: 每张图在自己相机系下的点图 (不依赖跨帧一致性)
* `conf`: 置信度 logit, 用 sigmoid 转成概率

注意: 当 $N>1$, $T_{wc}$ 会被强制改到「第 0 张图为世界原点」:
$$T'_{wc} = T_{wc}(0)^{-1} \cdot T_{wc}$$
所以输出的"世界系"就是第一帧相机系, 这一点在和其他 SLAM 系统 (DPVO/ORB-SLAM3) 衔接时要小心。

---

## 5. Mapping 阶段

Mapping 是 KV-Tracker 把 π³ "存进来" 的过程, 只在**关键帧集合**上做。

设关键帧集 $\{I_1, \ldots, I_N\}$, 每帧 token $X_n \in \mathbb{R}^{M \times d_k}$, 拼起来:

$$
X_{\text{map}} = [X_1; X_2; \ldots; X_N] \in \mathbb{R}^{NM \times d_k}.
$$

每个全局 attention block (一共 $L$ 个) 计算:

$$
Q_{\text{map}}^{(l)} = X_{\text{map}}^{(l)} W_Q^{(l)},
\quad
K_{\text{map}}^{(l)} = X_{\text{map}}^{(l)} W_K^{(l)},
\quad
V_{\text{map}}^{(l)} = X_{\text{map}}^{(l)} W_V^{(l)}.
$$

$$
Y_{\text{map}}^{(l)} = \operatorname{softmax}\!\left(\frac{Q_{\text{map}}^{(l)}{K_{\text{map}}^{(l)}}^\top}{\sqrt{d_k}}\right) V_{\text{map}}^{(l)}.
$$

特征解释:

* $W_Q^{(l)}, W_K^{(l)}, W_V^{(l)}$: 第 $l$ 个 global block 学到的线性投影矩阵, **整个推理过程不变**。
* "所有 $NM$ 个 token 互相看见": **full bidirectional attention**, 这是多视图一致性的来源。
* Mapping 一次的复杂度: $\mathcal{O}((NM)^2)$ — 关键帧多了之后非常昂贵, 但**只有添加关键帧时才做**。

Mapping 结束后, KV-Tracker 立刻把每一层的 $K^{(l)}_{\text{map}}, V^{(l)}_{\text{map}}$ 存下来:

$$
\mathcal{C} = \left\{(K^{(l)}_{\text{map}}, V^{(l)}_{\text{map}})\right\}_{l=1}^{L}.
$$

这就是**KV-cache 场景表示**。

---

## 6. Tracking 阶段

新来一帧 $I_q$, 编码得到 token $X_q \in \mathbb{R}^{M \times d_k}$。

#### Step 1: 只算新帧的 Q (以及它自己的 K/V, 用于 self-attn 部分)

$$
Q_q^{(l)} = X_q^{(l)} W_Q^{(l)},\qquad K_q^{(l)} = X_q^{(l)} W_K^{(l)},\qquad V_q^{(l)} = X_q^{(l)} W_V^{(l)}.
$$

#### Step 2: 把缓存的关键帧 K/V 和新帧的 K/V 拼起来 (但不 BP, 不更新)

$$
K_{\text{track}}^{(l)} = [K_{\text{map}}^{(l)}; K_q^{(l)}],
\quad
V_{\text{track}}^{(l)} = [V_{\text{map}}^{(l)}; V_q^{(l)}].
$$

#### Step 3: 只让新帧的 Q 去做 attention

$$
Y_q^{(l)} = \operatorname{softmax}\!\left(\frac{Q_q^{(l)}{K_{\text{track}}^{(l)}}^\top}{\sqrt{d_k}}\right) V_{\text{track}}^{(l)}.
$$

注意 $Y_q^{(l)} \in \mathbb{R}^{M \times d_k}$, 只有 $M$ 个 token, **不是 $(N{+}1)M$**。也就是说:

* Tracking 推理只更新新帧的特征。
* 关键帧的 K/V 不被任何东西改, 因此 KV-cache 不会"被污染"。

代码对照 (`pi3/models/pi3.py` 在 `kv` 分支):

```python
def forward(self, imgs, cam_only=False, store_cache=False, use_cache=False, tokens_mask=None, **kwargs):
    ...
    if cam_only:
        # 跳过 point_head / conf_head, 只走 camera_head, 进一步加速
        ...
```

`cam_only=True` 时连点图和置信度头都不算, 这就是 README 推荐的实时模式
`python main.py config/video.yaml --cam_only --resize_dim 308 --rerun` 背后做的事。

---

## 7. KV-cache 为什么能作为隐式场景表示

直觉解释:

* 全局 attention 中的 $K, V$ 实际编码了**"关键帧上哪些 patch 应该被注意到, 它们要传递什么信息"**。
* 这是网络在大数据上学到的"多视图一致性先验"在具体关键帧上的实例化。
* 一旦关键帧固定 + 网络权重固定, $K_{\text{map}}, V_{\text{map}}$ 就是关于这堆关键帧"该如何被检索"的**完整记忆**。
* 新帧 cross-attend 这堆记忆 ≈ "我在这堆已经被网络认知的场景里, 应该放在哪个位姿"。

为什么这样不会漂?

* 关键帧的 $K, V$ 永远不变 → 不会被低质量观测污染 (相比之下, 流式更新的 memory token 会随时间漂)。
* 新帧只读不写 → 错误不会反过来腐蚀地图。
* 关键帧是稀疏选的, 覆盖典型视角 → 整个观测空间被几个静态 KV 段"钉住"。

这就是 abstract 里那句话: **"This significantly improves the runtime inference speed, enabling 27 FPS tracking, a 15× speed-up over recomputing the keys and values for frames whose geometry we already know."**

---

## 8. 公式逐个解析

### 8.1 Scaled Dot-Product Attention (式 1)

$$
\operatorname{Attention}(Q, K, V) = \operatorname{softmax}\!\left(\frac{QK^\top}{\sqrt{d_k}}\right) V.
$$

* $Q \in \mathbb{R}^{n_q \times d_k}$: query — "当前 token 想找什么信息"
* $K \in \mathbb{R}^{n_k \times d_k}$: key — "每个候选 token 可被什么键索引"
* $V \in \mathbb{R}^{n_k \times d_v}$: value — "如果被检索到, 实际汇聚的信息"
* $\sqrt{d_k}$: 缩放, 防止内积过大导致 softmax 饱和
* softmax 沿 $n_k$ 维度归一化 → 得到注意力权重矩阵 $A \in \mathbb{R}^{n_q \times n_k}$
* 最终输出: $A V \in \mathbb{R}^{n_q \times d_v}$, 即每个 query 加权汇聚 value

### 8.2 Q/K/V 线性投影 (式 2)

$$
\operatorname{Proj}(X; \theta_l) = (Q, K, V) = (X W_Q^{(l)},\; X W_K^{(l)},\; X W_V^{(l)}).
$$

* $X \in \mathbb{R}^{n \times d}$: 输入 token (一帧或多帧拼起来)
* $W_Q^{(l)}, W_K^{(l)}, W_V^{(l)} \in \mathbb{R}^{d \times d_k}$: 第 $l$ 层学到的线性投影 (实现里通常合成一个 `qkv` 大矩阵)
* $\theta_l$ 在论文里代表 layer $l$ 的所有可学参数

### 8.3 Mapping 全局自注意力

$$
X_{\text{map}} = [X_1, X_2, \ldots, X_N] \in \mathbb{R}^{NM \times d}.
$$

$$
Q_{\text{map}}^{(l)} = X_{\text{map}}^{(l)} W_Q^{(l)},\;
K_{\text{map}}^{(l)} = X_{\text{map}}^{(l)} W_K^{(l)},\;
V_{\text{map}}^{(l)} = X_{\text{map}}^{(l)} W_V^{(l)}.
$$

$$
Y_{\text{map}}^{(l)} = \operatorname{softmax}\!\left(\frac{Q_{\text{map}}^{(l)}{K_{\text{map}}^{(l)}}^\top}{\sqrt{d_k}}\right) V_{\text{map}}^{(l)}.
$$

* 关键: 注意力矩阵 size = $NM \times NM$ — 每个 token 对所有 token (包括其他关键帧) 加权。
* 这就是为什么"所有关键帧之间互相看见 + 全局一致"。

### 8.4 KV-cache 定义

Mapping 完成后保存:

$$
\mathcal{C} = \left\{K_{\text{map}}^{(l)},\; V_{\text{map}}^{(l)}\right\}_{l=1}^{L},
$$

* $l \in \{1, \ldots, L\}$ 是全局 attention 层的索引 (在代码里是 `(i % 2) != 0` 的 decoder block)。
* $L$ 通常是 $L_{\text{decoder}}/2$ (frame-wise 和 global 交替)。

### 8.5 Tracking 的 cross-attention

新帧 $I_q$ 编码为 $X_q \in \mathbb{R}^{M \times d}$。

只为新帧算 Q:
$$
Q_q^{(l)} = X_q^{(l)} W_Q^{(l)}.
$$

把新帧自己的 K/V 拼进 cache (只在这次推理里临时拼, 不存):
$$
K_{\text{track}}^{(l)} = [K_{\text{map}}^{(l)};\; K_q^{(l)}],\quad
V_{\text{track}}^{(l)} = [V_{\text{map}}^{(l)};\; V_q^{(l)}].
$$

cross-attention:
$$
Y_q^{(l)} = \operatorname{softmax}\!\left(\frac{Q_q^{(l)}{K_{\text{track}}^{(l)}}^\top}{\sqrt{d_k}}\right) V_{\text{track}}^{(l)}.
$$

注意:

* attention 矩阵 size = $M \times (NM + M)$, 比 Mapping 的 $(NM)^2$ 小很多。
* $Y_q$ 只有 $M$ 个 token, 不更新关键帧 token。

### 8.6 复杂度对比

| 阶段 | attention 输入 | 复杂度 |
| :--- | :--- | :--- |
| Mapping 全图 | $X_{\text{map}} \in \mathbb{R}^{NM \times d}$ | $\mathcal{O}((NM)^2 \cdot d)$ |
| Mapping 每加一个关键帧 (naive) | 重算所有 $N$ 帧 | $\mathcal{O}((NM)^2 \cdot d)$ |
| Tracking 一帧 (本方法) | Q 是新帧 $M$ token, KV 是 $NM + M$ | $\mathcal{O}(M \cdot NM \cdot d) = \mathcal{O}(NM^2 d)$ |

随关键帧数 $N$:

* 朴素方法每帧 $\mathcal{O}(N^2 M^2 d)$ — 二次增长。
* KV-Tracker 每帧 $\mathcal{O}(NM^2 d)$ — **线性**增长。

实测加速比 15× (paper 数据), 27 FPS 实时 tracking。

### 8.7 位姿输出格式

$$
T_{wc} = \begin{bmatrix} R_{wc} & t_{wc} \\ \mathbf{0}^\top & 1 \end{bmatrix} \in SE(3).
$$

* $R_{wc} \in SO(3)$: 相机系到世界系的旋转
* $t_{wc} \in \mathbb{R}^3$: 相机光心在世界系的位置 (是 $T_{wc}[:3, 3]$)

在 KV-Tracker 中, 世界系**=第一个关键帧的相机系** (因为 `pi3_inference()` 显式做了 $T'_{wc} = T_{wc}(0)^{-1} T_{wc}$)。

---

## 9. 关键帧策略

代码里 (`main.py`) 有两种触发模式:

#### Scene 模式 (默认, `--obj_mode` off)

```python
should_add_kf = (int(args.kf_auto) > 0 and (idx % int(args.kf_auto) == 0))
should_add_kf = should_add_kf and (kf_rgb_np.shape[0] < 20)
```

* 每 `kf_auto` 帧 (默认 50) 加一个关键帧
* 关键帧总数 hard cap 20 (避免 attention 复杂度爆炸)

#### Object 模式 (`--obj_mode`)

```python
should_add_kf = check_if_keyframe(obj_center, cur_T_wc, kf_T_wc)
```

`check_if_keyframe` 计算当前 view direction (相机到 obj 中心) 的方位角/仰角, 与所有现有关键帧比对, 任一角度超过阈值 (默认 10°) 就触发:

$$
\Delta_\text{elev} = \min_i |elev_q - elev_i|,\quad
\Delta_\text{azim} = \min(|azim_q - azim_i|,\; 360°-|azim_q - azim_i|).
$$

$$
\text{add kf} \iff \Delta_\text{elev} > 10° \;\lor\; \Delta_\text{azim} > 10°.
$$

#### Confidence reject

新加的关键帧如果 confidence 太低, 还可以被"反悔" (代码里 `revert_kf`), 防止地图被破帧污染。

---

## 10. Object-level Tracking 与 SAM2

Object 模式下流程多一步分割:

1. SAM2 实时分割当前帧, 输出 binary mask。
2. 把 mask 内 token 当 "object token", mask 外 token 在 attention 里被 mask 掉 (`tokens_mask`)。
3. Pi3 推理时只对 object 区域计算几何 → object-level pose / pointcloud。
4. obj_center = mask 内点云均值, 用于关键帧策略 (式见 §9)。

为什么需要 SAM2 而不是用 bbox? 因为 π³ 是 patch-based, mask 给到 token-level 才能精确把背景排除掉, 否则 attention 会被无关 token 干扰。

代码对照: `kv_tracker/sam_interface.py` 提供 `SAMInterface`, 几乎所有 dataloader (TUMLoader / phoneLoader / arcticLoader / onePoseLoader 等) 都继承它。

---

## 11. Confidence 的意义

π³ 的 `conf_head` 输出每像素一个 logit, 经 sigmoid 得 $C \in [0,1]^{H \times W}$。它表示**网络对该像素 3D 预测的不确定性**。

KV-Tracker 用它做两件事:

1. **关键帧选择**: 新关键帧的 mean confidence 必须 $\ge$ 某阈值才接受, 否则 `revert_kf` (object 模式)。
2. **点云筛选**: 输出点云时只保留 $C(u, v) > $ threshold 的点, 避免噪声点云。代码里阈值定义为:
   $$
   \text{kf\_conf\_thresh} = 0.6 \cdot \bar{C}_{\text{first kf}},
   \quad
   \text{pts3d\_conf\_thresh} = 1.15 \cdot \text{kf\_conf\_thresh}.
   $$
3. **死帧检测**: 如果当前帧 mean conf < `kf_conf_thresh * 0.3`, 认为 "失联", 用上一帧位姿代替:
   ```python
   if conf < kf_conf_thresh * 0.3:
       pred_T_wc[0, 0] = prev_pred   # 跳过当前预测, 用上一帧位姿
   ```

---

## 12. 和 ORB-SLAM3、DPVO、VGGT、DUSt3R 的关系

| 方法 | 前端 | 后端 | 地图 | 实时 | 闭环 | 单目尺度 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| ORB-SLAM3 | ORB 特征 + 描述子 | g2o / GTSAM 局部+全局 BA | 稀疏点云 + 关键帧 | ✅ | ✅ (DBoW2) | 单目无尺度, 双目/RGBD/惯性可 |
| DPVO | 学习 patch + RAFT-like 光流 | DBA | 稀疏 patch 点云 | ✅ | 可选 (默认关) | 单目无尺度 |
| DUSt3R | ViT, 两图配对 | 全局对齐 | 稠密点图 | ❌ (离线) | ❌ | 单目无尺度 |
| VGGT | π³-style ViT, N 图全局 | 一次前馈 | 稠密点图 | ❌ (推理重) | ❌ | 单目无尺度 |
| **KV-Tracker** | **π³ (Pi3_w_KV) + KV cache** | **关键帧 mapping + cross-attn tracking** | **关键帧 + 稠密点图 + KV cache** | **✅ 27 FPS** | **✅ (KV-cache 永久保留)** | 单目无尺度 |

最大差异:

* ORB-SLAM3/DPVO 是「特征/光流 + 几何优化」, 没用大模型多视图先验。
* DUSt3R/VGGT 是 KV-Tracker 的"上游" (同一族网络), 但只支持离线/批量推理。
* KV-Tracker 把 VGGT 这类方法做成了 online: 把昂贵的 mapping 阶段稀疏化 (只在关键帧上做) + 用 KV-cache 复用结果。

闭环: KV-Tracker 是**隐式闭环**——只要旧视角的关键帧还在 cache 里, 新帧 cross-attn 就会自然把当前位姿对回旧地图。这点和 ORB-SLAM3 显式的 DBoW2 + Sim(3) 回路检测完全不同。

---

## 13. 实验结果怎么看

论文实验主要在 4 个数据集:

* **7-Scenes** (室内单房间): scene-level tracking, 对比 ORB-SLAM3 / GLACE / Pi3-original。 KV-Tracker 在多个房间上 ATE 提升, 同时 FPS 高 (15-27 FPS, 比朴素 Pi3 快 15×)。
* **TUM RGB-D** (经典 RGB-D 数据集): scene-level, KV-Tracker 跑纯 RGB 模式也能匹敌许多 RGB-D 方法。
* **Arctic** (双手物体交互, 自遮挡严重): object-level tracking 测试。
* **OnePose / OnePose++**: 物体 6-DoF tracking, 不需要 CAD。

值得关注的指标:

* **ATE / RPE**: 标准轨迹误差。
* **FPS**: 强调实时性。论文给的是 ~27 FPS @ $308 \times 308$ resize。
* **Speedup**: 与原始 π³ 比 15×。

---

## 14. 局限性

论文和代码里能看到的限制:

1. **尺度**: 纯单目 KV-Tracker 没有真实米制尺度 (和 DUSt3R/VGGT 一样)。
2. **关键帧上限**: 代码里硬限 20 帧, 超过会拒绝。这是因为关键帧 mapping 的 $\mathcal{O}((NM)^2)$ 复杂度仍然在。长序列需要替换/淘汰策略 (论文未深入)。
3. **显存**: KV-cache 全部驻留显存 (每层每个 token 都有 K, V), 关键帧上限和显存挂钩。
4. **没有 BA**: KV-cache 是网络隐式记忆, 没有显式 bundle adjustment, 所以**关键帧位姿一旦写入就被冻结**, 无法事后做全局优化。
5. **跨片段拼接**: 多视频片段拼成大地图的能力没演示, 当前是单 session online。
6. **闭环依赖关键帧覆盖**: 如果回到一个完全新视角 (与所有关键帧都差很远), 就没有 cache 可查 → 该模式效果未知。

---

## 15. 对商场单目视觉导航的启发

商场场景特点:

* 走廊很长 (几十米), 视觉重复度高 (相邻店铺看起来相似)。
* 视频 30 FPS 持续 1-2 分钟, 关键帧上限 20 远不够。
* 行人是动态干扰物。

哪些 KV-Tracker 思想可以直接借用:

1. **关键帧 + 隐式记忆**: 比传统稀疏点云图更紧凑, 适合做 "VPR + 重定位" 的离线场景表示。
2. **复用 Pi3 / VGGT 等多视图先验**: 比从 0 训 ORB-SLAM3 的 BoW 词典灵活, 适合大规模商场预扫描后部署。
3. **置信度筛选**: π³ confidence 可以直接做 BEV 上的"可信观测掩膜", 比 ORB 特征的"内点比例"语义更丰富。

哪些不适合直接用:

1. **关键帧 cap 20**: 商场太大, 需要扩展淘汰策略 (e.g. LRU + 当前视角附近邻区 keep)。
2. **显式尺度**: 必须和 IMU/DPVO/已知地砖尺寸或 CAD 配准对齐, 否则米制 BEV 无意义。
3. **动态物体**: π³ 假设场景静态, 商场行人需要前置 mask (SAM2 / YOLO-seg) 把动态 token 排除掉, 否则会污染 KV-cache。
4. **多楼层**: 单 session KV cache 不直接支持多楼层切换, 需要外加 place recognition 决定何时切换 cache。

如何接到现有 `src/people_bev_tracker` 流水线: 详见 [`KV_Track3r_商场导航与BEV地图应用方案.md`](./KV_Track3r_商场导航与BEV地图应用方案.md) §3。

---

## 附录: 代码结构速查

```
project code/KV-tracker/kv_tracker-main/
├── main.py                       # 入口, run_track3r()
├── config/
│   ├── video.yaml                # 自定义视频
│   ├── live.yaml                 # RealSense 实时
│   └── tum.yaml / 7-scenes.yaml  # 数据集
├── kv_tracker/
│   ├── pi3_utilts.py             # load_pi3_from_pretrained, pi3_inference
│   ├── rerun_tools.py            # rr_viz_cam, rr_viz_pose
│   ├── sam_interface.py          # SAM2 wrapper
│   ├── live_cap.py               # RealSense capture
│   ├── geometry.py               # umeyama_alignment
│   └── dataloaders/
│       ├── phone.py              # 视频文件 loader (我们用这个)
│       └── ...
└── thirdparty/
    ├── Pi3/                      # Pi3_w_KV (kv branch)
    └── segment-anything-2-real-time/
```

main.py 关键变量:

| 变量 | 含义 | 涉及行 |
| :--- | :--- | :--- |
| `frames_que` | 跨进程帧队列 | 250 |
| `model` | π³ KV 版本 | 258-259 |
| `keyframes` | 关键帧列表 | 267-275 |
| `batch_pts3d, batch_pred_T_wc, batch_conf` | mapping 输出 | 298-300, 464-466 |
| `origin_offset` | 世界系归一化 | 298 |
| `poses_np_list` | 全部 tracking 位姿 | 348, 422-424 |
| `kf_poses` | 关键帧位姿 | 351, 696-699 |
| `kf_idx` | 关键帧对应的源帧编号 | 350, 701-702 |
| `pcd_list` | 局部点云段 | 347, 426-429 |
| `current_sim3_R/t/s` | 关键帧重建后 Sim(3) 对齐 | 334-336, 504-526 |
| `kf_conf_thresh, pts3d_conf_thresh` | confidence 阈值 | 315-316 |

---

## 16. 一句话回顾

> **"关键帧的 K/V 是网络对场景的隐式记忆。让新帧只读不写, 复杂度从 $\mathcal{O}((NM)^2)$ 降到 $\mathcal{O}(NM^2)$, 把离线多视图网络变成在线 6-DoF tracker。这是 KV-Tracker 的全部。"**
