# people_bev_tracker

行人 BEV 跟踪的离线流水线 (第一版)：

    原始视频 -> Ultralytics YOLO-seg + 内置 BoT-SORT/ByteTrack
        -> 行人脚底像素 -> DPVO 相机位姿 + 相机内参 + 地面平面
        -> 行人世界坐标 -> BEV 平面 + EMA 平滑
        -> 二维 BEV 平面图 + JSON 记录

> 本目录与 `src/kv_tracker/` 完全独立，互不调用。

## 依赖

```bash
pip install -e thirdparty/official/ultralytics
pip install -r src/people_bev_tracker/requirements.txt
```

CUDA 版 PyTorch 已经被 `dpvo` 环境提供，无需重复安装。

## 输入

| 项 | 路径 | 说明 |
| --- | --- | --- |
| 视频 | `resources/input_video.mp4` | 原始未叠加 mask 的输入视频 |
| 相机标定 | `config/KannalaBrandt8_1280x720.yaml` | 自动按视频分辨率缩放 K |
| DPVO 轨迹 | `output/dpvo/trajectory_tum.txt` | TUM 8 列格式 |

如果 DPVO 轨迹不存在，请先跑 DPVO：

```bash
cd "project code/DPVO"
python demo.py \
  --imagedir /home/ros/ros2_orbslam3/resources/input_video.mp4 \
  --calib    calib/custom_mall.txt \
  --name     input_video_clean \
  --stride   2 \
  --save_trajectory
cp saved_trajectories/input_video_clean.txt /home/ros/ros2_orbslam3/output/dpvo/trajectory_tum.txt
```

## 短测试

```bash
python src/people_bev_tracker/scripts/offline_pipeline.py \
  --video resources/input_video.mp4 \
  --calib config/KannalaBrandt8_1280x720.yaml \
  --pose output/dpvo/trajectory_tum.txt \
  --output-dir output/people_bev_test \
  --max-frames 30
```

## 完整运行

```bash
python src/people_bev_tracker/scripts/offline_pipeline.py \
  --video resources/input_video.mp4 \
  --calib config/KannalaBrandt8_1280x720.yaml \
  --pose output/dpvo/trajectory_tum.txt \
  --output-dir output/people_bev
```

## 输出

* `output/people_bev/bev_tracking.mp4` —— 二维 BEV 平面图视频，叠加相机轨迹 + 行人位置 + track_id
* `output/people_bev/debug_overlay.mp4` —— 原始视频上叠加 bbox / track_id / 脚底点 / 世界坐标
* `output/people_bev/people_tracks.json` —— 每一帧每个 track 的 bbox / foot_pixel / world_xyz / bev_xy
* `output/people_bev/camera_trajectory.json` —— 每一帧匹配到的 T_wc + BEV 投影

## 当前已知限制

1. 单目 DPVO 没有真实米制尺度，BEV "米尺" 仅是 DPVO 尺度。可调 `pose.scale`。
2. 地面假设 `y=0`、`normal=[0,1,0]`，未做地面拟合。若投影明显不合理，可改成 `normal=[0,0,1]`、`bev_axes=["x","y"]`。
3. 鱼眼 / KannalaBrandt 畸变本版未真正反畸变，仅按 Pinhole 模型处理。
4. 没有用 SAM2，只用 YOLO-seg；mask 比较粗。
5. 没有 ROS2 节点；只是离线 Python 脚本。
