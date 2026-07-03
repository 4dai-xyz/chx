# 路线方案评估与选择报告：DPVO 主轨迹下的商场二维地图落地

## 0. 结论先行

当前工程最合适的主路线是：

```text
短期 V1:
  DPVO 主轨迹
  + YOLO-seg / BoT-SORT 行人检测跟踪
  + 现有 people_bev_tracker 脚底点地面投影
  + 自研轻量二维栅格层
  -> 离线输出 BEV 视频和 JSON

中期 V2:
  在 DPVO 主轨迹不变的前提下
  + Depth Anything V2 / Metric Depth 关键帧深度
  + 动态行人 mask 剔除
  + 深度反投影点云
  + 单目尺度用相机高度/地面约束对齐
  -> 更可靠的障碍和可通行区域

后期 V3:
  ROS2 在线化
  DPVO node 发布 /odom /tf
  YOLO tracker node 发布 /people
  depth/mapper node 发布 /map 或 /bev/map_image
  可选接 Nav2 costmap_2d / pointcloud_to_laserscan / OctoMap
```

不建议第一版直接接 Cartographer / Nav2 costmap_2d / OctoMap 当主建图后端。原因是：你现在是纯单目，深度和尺度还没有稳定闭环；过早接重型 ROS 导航栈，会把主要问题从“几何是否正确”变成“系统集成和调参”，开发成本高，定位问题也更难。

如果只回答“哪个方案更适合当前工程”：

```text
当前最适合:
  DPVO 主轨迹 + 自研轻量 BEV occupancy grid + YOLO/BoT-SORT 动态行人层

下一阶段最值得加:
  Depth Anything V2 关键帧深度建图

暂缓:
  Cartographer / Nav2 costmap_2d / OctoMap 全量接入

如果未来换 RGB-D 或 LiDAR:
  OctoMap / Voxblox / Nav2 costmap_2d 会立刻变成更优方案
```

---

## 1. 参与比较的方案

### 方案 0：当前路线 A

```text
DPVO trajectory_tum.txt
  + YOLO-seg / BoT-SORT
  + 脚底点地面投影
  + 自研 BEVCanvas
  + 自研 static_map
```

当前已有基础：

```text
src/people_bev_tracker/
output/dpvo/trajectory_tum.txt
project code/DPVO/mall_dpvo.ply
```

目标输出：

```text
output/route_A/bev_tracking_route_A.mp4
output/route_A/static_map.png
output/route_A/people_tracks_route_A.json
output/route_A/camera_trajectory_route_A.json
```

### 方案 1：同事建议的 ROS2 全模块架构

```text
DPVO ROS node
  -> /odom /tf

YOLOv8 + ByteTrack node
  -> /people/detections

Depth Anything / ZoeDepth node
  -> /depth

pointcloud_to_laserscan / costmap_2d / Cartographer
  -> /map
```

这是长期正确的系统架构，但不适合作为当前第一版的最小落地路径。

### 方案 2：单目 + Depth Anything V2 + 自研 Occupancy

```text
DPVO 位姿
  + Depth Anything V2 深度
  + YOLO mask 剔除动态人
  + 相机高度/地面约束恢复尺度
  + 自研 occupancy grid
```

这是我推荐的 V2 路线。它比纯 DPVO 点云更能表达墙、柜台、货架、柱子等静态结构，但仍保持可控，不必立刻引入完整 Nav2/OctoMap。

### 方案 3：单目 + Depth Anything V2 + ROS 标准后端

```text
DPVO /tf
  + Depth Anything 点云
  + pointcloud_to_laserscan
  + Nav2 costmap_2d 或 Cartographer
```

优点是标准化，后续导航接入顺滑。缺点是：单目伪深度不稳定时，标准后端会把伪影认真地融合进地图，调参和排错成本很高。

### 方案 4：RGB-D / LiDAR + OctoMap / Voxblox / costmap_2d

如果硬件升级到 RGB-D、双目或 LiDAR，这是最稳的工程路线。但当前用户明确是单目纯视觉，所以只能作为未来硬件升级方案。

---

## 2. 维度对比

| 维度 | 当前路线 A | ROS2 全模块 | Depth Anything + 自研 grid | Depth + Nav2/costmap | RGB-D/LiDAR + OctoMap |
|---|---:|---:|---:|---:|---:|
| 当前可落地性 | 高 | 中低 | 中 | 中低 | 低，需硬件 |
| 代码改造量 | 低 | 高 | 中 | 高 | 高 |
| 对现有 DPVO 轨迹复用 | 高 | 高 | 高 | 高 | 中 |
| 实时性 | 中高 | 中 | 中 | 中 | 高 |
| 静态障碍质量 | 中低 | 取决于深度 | 中高 | 中高 | 高 |
| 可通行区域可靠性 | 中 | 中 | 中高 | 中高 | 高 |
| 尺度问题 | 未完全解决 | 未完全解决 | 可通过地面/高度解决 | 可通过地面/高度解决 | 天然较好 |
| 动态行人展示 | 已有基础 | 标准化好 | 好 | 好 | 好 |
| 排错难度 | 低 | 高 | 中 | 高 | 中 |
| 适合作为第一版 | 最适合 | 不适合 | 次优 | 暂缓 | 不适用当前硬件 |

判断：

```text
第一版要“尽快看到相机轨迹 + 行人 + 地图层”，当前路线 A 最合适。
第二版要“障碍和可通行区域更像真实商场”，Depth Anything V2 + 自研 grid 最合适。
第三版要“导航栈标准化和 ROS 在线运行”，再接 Nav2 costmap_2d / pointcloud_to_laserscan。
```

---

## 3. 为什么不建议现在直接上 Cartographer / costmap_2d / OctoMap

### 3.1 Cartographer 更适合真实 2D LiDAR

Cartographer 的强项是 2D/3D 激光 SLAM。你可以用 `pointcloud_to_laserscan` 把点云拍成 LaserScan，但当前输入是单目伪深度，误差模式和真实 LiDAR 不一样。

单目伪深度问题：

```text
1. 没有天然米制尺度。
2. 玻璃、反光地砖、橱窗会产生深度幻觉。
3. 纯旋转/弱纹理时深度投影不稳定。
4. 深度网络的远近关系不等于真实几何可通行性。
```

所以，当前阶段用 Cartographer 很可能把“深度网络的幻觉”融合成很正式的地图，看起来工程化，实际难排错。

### 3.2 Nav2 costmap_2d 很适合后期，但第一版太重

Nav2 的 Obstacle Layer 支持 `LaserScan` 或 `PointCloud2` 数据源，并有 `min_obstacle_height`、`max_obstacle_height`、`marking`、`clearing`、`obstacle_max_range`、`raytrace_max_range` 等参数。这个思想非常适合后期在线导航。

但 costmap_2d 默认假设传感器观测比较可靠。现在你的“传感器”实际上是：

```text
RGB 单目 + 深度网络 + DPVO 尺度恢复
```

先自研一个轻量 grid，可以清楚看见每一步几何是否正确：

```text
深度图是否靠谱
尺度是否正确
地面是否正确
障碍高度阈值是否正确
free corridor 是否合理
```

等这些稳定后，再把自研 grid 的逻辑迁移到 Nav2 costmap layer。

### 3.3 OctoMap/Voxblox 更适合有真深度传感器

OctoMap 的 ROS stack 包含 `octomap_server`，它是接收真实点云/深度流做三维占据建图的成熟方案。RGB-D 或 LiDAR 场景里它很好用。

但纯单目伪深度下，OctoMap 会持续融合错误深度，导致错误障碍被“固化”。如果没有可靠 clearing 机制和动态 mask，商场里行人、玻璃、反光都会污染地图。

结论：

```text
有 RGB-D/LiDAR:
  OctoMap/Voxblox 是好选择。

当前纯单目:
  暂不作为第一版主后端。
```

---

## 4. 对同事建议的逐项评估

### 4.1 “用 ROS2 做系统胶水”

评价：长期正确，短期分阶段。

建议：

```text
短期:
  保持 people_bev_tracker 离线脚本，先把几何链路跑通。

中期:
  把 DPVO 封装成真正订阅 /camera/image_raw 的 ROS2 node，发布 /odom /tf。

后期:
  YOLO tracker、depth mapper、BEV renderer 都拆成 ROS2 nodes。
```

原因：

```text
当前 DPVO 封装 run_dpvo_video.py 不是在线 topic 节点，而是视频文件 runner。
直接强行 ROS2 化会引入大量同步、QoS、GPU 调度和延迟问题。
```

### 4.2 “YOLOv8 + ByteTrack”

评价：方向正确，但你当前更适合继续用 YOLO-seg + BoT-SORT/ByteTrack 可切换。

当前代码已经有：

```text
src/people_bev_tracker/people_bev_tracker/person_yolo_tracker.py
```

它使用 Ultralytics：

```text
YOLO segmentation
model.track(..., tracker="botsort.yaml")
```

建议：

```text
默认 BoT-SORT:
  移动相机环境更稳，ID 切换更少。

fallback ByteTrack:
  更轻更快，适合检测稳定且遮挡不严重。
```

YOLOv8/YOLO11 不需要纠结，Ultralytics 接口统一。关键是：

```text
必须使用 segmentation mask 或至少 bbox bottom footpoint。
必须持久化 track_id。
必须把人从静态建图深度中剔除。
```

### 4.3 “空间投影结合相机内参和深度”

评价：对障碍建图是必须的；对行人定位第一版不一定必须用深度。

行人位置：

```text
第一版继续用脚底点 + 地面平面射线求交。
```

这样比直接用深度网络估计人框深度更稳，因为人的身体深度不等于脚底地面点，深度网络在人身上也可能不准。

障碍地图：

```text
需要深度或点云。
DPVO 稀疏点云可先做粗糙障碍。
Depth Anything V2 可作为 V2 生成更密点云。
```

### 4.4 “单目必须外挂 Depth Anything / ZoeDepth”

评价：中期正确，但不是第一版阻塞项。

第一版可以先用：

```text
project code/DPVO/mall_dpvo.ply
```

做静态障碍粗图，同时用相机走过路径做 free corridor。

V2 再加入：

```text
Depth Anything V2 metric / relative depth
```

Depth Anything V2 官方提供多种模型规模，从 Small 到 Large，并强调 V2 相比 V1 在细节和鲁棒性上更强，也有 metric depth 分支可用。商场这种玻璃、反光、弱纹理环境，Depth Anything V2 比传统几何深度更值得尝试。

### 4.5 “pointcloud_to_laserscan + costmap_2d”

评价：后期在线化推荐，不是当前离线第一版。

`pointcloud_to_laserscan` 的 ROS2 包用途非常贴合：它把 `sensor_msgs/PointCloud2` 投影成 `sensor_msgs/LaserScan`，并支持 `min_height`、`max_height`、`target_frame` 等参数。后续如果我们有稳定点云流，这个包可以直接接。

但当前实现离线 BEV 时，自研投影更直接：

```text
points.npy / PLY
  -> height filtering
  -> 2D histogram
  -> static_map.png
```

等这个逻辑稳定后，再用 `pointcloud_to_laserscan` 做 ROS2 版本。

### 4.6 “OctoMap / Voxblox”

评价：如果换 RGB-D/LiDAR，非常推荐；纯单目当前暂缓。

OctoMap 的 `octomap_mapping` 是成熟 ROS stack，包含 `octomap_server`。它适合接收真实深度/点云做三维占据地图，并可保存 `.bt` / `.ot` 地图。

但对于当前纯单目：

```text
Depth Anything 深度是预测，不是测距。
OctoMap 会把预测错误递归融合，错了以后不容易清除。
```

所以 OctoMap 放到：

```text
硬件升级到 RGB-D / LiDAR
或 Depth Anything + 尺度恢复已经非常稳定
```

之后再接。

---

## 5. 推荐最终架构

### 5.1 V1：当前最适合的离线架构

```text
resources/input_video.mp4
    │
    ├── DPVO
    │     └── output/dpvo/trajectory_tum.txt
    │
    ├── YOLO-seg + BoT-SORT
    │     └── track_id + mask + bbox + footpoint
    │
    ├── DPVO PLY / optional VGGT PLY
    │     └── static occupancy grid
    │
    └── people_bev_tracker
          ├── 相机轨迹层
          ├── 静态栅格层
          └── 动态行人层
```

输出：

```text
output/route_A/bev_tracking_route_A.mp4
output/route_A/static_map.png
output/route_A/people_tracks_route_A.json
output/route_A/camera_trajectory_route_A.json
```

优点：

```text
1. 复用现有代码最多。
2. 不引入大型 ROS 导航栈。
3. 几何错误容易定位。
4. 能最快产出可展示结果。
```

### 5.2 V2：加入 Depth Anything V2 的关键帧建图

新增：

```text
depth_mapper.py
depth_scale_align.py
keyframe_selector.py
```

流程：

```text
DPVO 轨迹选择关键帧
  -> YOLO mask 剔除行人
  -> Depth Anything V2 预测深度
  -> 用地面/相机高度估计 scale
  -> 深度反投影成点云
  -> 投影成 static_map
```

关键点：

```text
深度不需要每帧跑。
每秒 2-3 个关键帧足够建静态图。
DPVO 继续高频跑，用于相机轨迹。
```

推荐模型：

```text
Depth-Anything-V2-Small:
  先跑通，速度快。

Depth-Anything-V2-Base/Large:
  离线质量更好，显存更高。

Metric Depth 分支:
  如果 indoor metric 效果稳定，优先用于尺度恢复。
```

### 5.3 V3：ROS2 在线化

节点建议：

```text
video_publisher_node
  -> /camera/image_raw

dpvo_node
  -> /odom
  -> /tf
  -> /dpvo/keyframe

yolo_tracker_node
  -> /people/tracks
  -> /people/masks

depth_mapper_node
  -> /static/cloud
  -> /static/occupancy_grid

people_projector_node
  -> /people/world_positions

bev_renderer_node
  -> /bev/map_image
  -> /bev/markers
```

此时可以考虑：

```text
pointcloud_to_laserscan
Nav2 costmap_2d
OctoMap
```

但它们应作为“ROS 标准化后端”，不是几何验证的第一步。

---

## 6. 对当前 route_A 执行报告的调整建议

现有：

```text
src/people_bev_tracker/docs/02_RouteA_DPVO主轨迹执行任务书.md
```

建议保持不变的点：

```text
1. DPVO 主轨迹。
2. people_bev_tracker 作为第一版主应用。
3. 自研 static_map 而不是直接接 costmap_2d。
4. KV/VGGT 只做增强。
5. 输出 output/route_A。
```

建议追加的 V2 任务：

```text
1. 新增 depth_anything_mapper.py。
2. 新增 keyframe_selector.py。
3. 新增 scale_from_ground.py。
4. 新增 depth_points_to_static_map.py。
5. 对比三种 static map:
   - DPVO PLY only
   - VGGT aligned PLY
   - Depth Anything V2 keyframes
```

建议暂缓的任务：

```text
1. pointcloud_to_laserscan。
2. Cartographer。
3. Nav2 costmap_2d。
4. OctoMap。
5. Voxblox。
```

---

## 7. 技术细节建议

### 7.1 行人定位不要直接用深度网络

行人位置应继续使用：

```text
person mask/bbox -> footpoint -> camera ray -> ground intersection
```

原因：

```text
1. 行人身体表面的深度不是脚底位置。
2. 深度网络对动态人会有边缘毛刺。
3. 地面约束让位置更稳定。
```

### 7.2 静态建图必须剔除动态人

深度图进入点云前：

```text
YOLO/SAM mask 把 person 区域置 invalid
```

否则行人会在 occupancy grid 里留下移动障碍残影。

### 7.3 free 不等于 “没有点”

单目点云很稀疏，没点不代表可通行。

第一版 free 应来自：

```text
相机走过路径附近 corridor
```

中期可以加入：

```text
地面分割区域反投影得到 observed-free
```

### 7.4 occupied 高度带要按 DPVO 尺度标定

当前 DPVO 没有米制尺度，所以：

```text
height_range: [0.02, 0.35]
```

这类参数都是 DPVO 单位，不是真实米。

V2 可用相机高度恢复尺度：

```text
scale = H_real / H_dpvo
```

### 7.5 玻璃和反光要单独处理

商场玻璃/橱窗是纯视觉地图的大坑：

```text
深度网络可能把玻璃后的物体当成可到达空间。
ORB/DPVO 可能在反光上产生不稳定特征。
```

建议 V2 增加：

```text
1. 深度置信度过滤。
2. 多关键帧一致性过滤。
3. 只把多次观测一致的点写入 occupied。
4. 单帧深度点不要立即固化到 static map。
```

---

## 8. 推荐实施顺序

### 阶段 1：当前 route_A V1

目标：

```text
DPVO 轨迹 + DPVO PLY + YOLO 行人 + 自研 static_map
```

执行：

```text
按 02_RouteA_DPVO主轨迹执行任务书.md 执行。
```

验收：

```text
output/route_A/bev_tracking_route_A.mp4
```

### 阶段 2：Depth Anything V2 离线增强

新增输出：

```text
output/route_A_depth/
├── depth_keyframes/
├── depth_points.npy
├── depth_static_map.png
├── bev_tracking_depth_route_A.mp4
└── depth_validation_report.md
```

验收：

```text
障碍轮廓比 DPVO PLY 更清楚。
行人不会污染静态地图。
相机轨迹仍使用 DPVO。
```

### 阶段 3：ROS2 在线架构

只在 V1/V2 验证通过后做。

目标：

```text
实时 /bev/map_image
实时 /people/world_positions
实时 /odom /tf
```

### 阶段 4：Nav2 / costmap / OctoMap

只有当深度/点云稳定后才接。

目标：

```text
nav_msgs/OccupancyGrid /map
Nav2 global_costmap/local_costmap
可选路径规划
```

---

## 9. 最终建议

### 最适合当前工程的选择

```text
继续当前 02_RouteA_DPVO主轨迹执行任务书.md。
不要立刻切到 Cartographer / costmap_2d / OctoMap。
不要让 KV-Tracker 轨迹参与主位姿。
```

### 最值得新增的能力

```text
Depth Anything V2 关键帧深度建图。
```

### 最值得保留的工程思想

同事建议中的模块解耦是正确的：

```text
DPVO 只负责高频位姿。
YOLO/BoT-SORT 只负责动态目标。
Depth/Map 模块只负责静态结构。
BEV renderer 只负责展示。
```

但实现顺序要反过来：

```text
先离线单进程验证几何。
再拆 ROS2 节点。
最后接 Nav2/OctoMap。
```

### 一句话路线

```text
现在走 DPVO + 自研轻量 BEV 栅格。
下一步加 Depth Anything V2。
再下一步 ROS2 在线化。
最后才接 Nav2 costmap 或 OctoMap。
```

---

## 10. 参考资料

1. Depth Anything V2 官方仓库  
   https://github.com/DepthAnything/Depth-Anything-V2

2. ROS2 pointcloud_to_laserscan 官方仓库  
   https://github.com/ros-perception/pointcloud_to_laserscan

3. Nav2 Obstacle Layer 参数文档  
   https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html

4. OctoMap ROS mapping 官方仓库  
   https://github.com/OctoMap/octomap_mapping
