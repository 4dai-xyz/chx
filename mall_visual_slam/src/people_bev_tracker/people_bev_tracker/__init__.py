"""people_bev_tracker — 行人 BEV 跟踪。

第一版离线流水线：
    原始视频 -> Ultralytics YOLO-seg + BoT-SORT/ByteTrack
        -> 行人脚底像素 + DPVO 位姿 + 地面投影
        -> 二维 BEV 平面图与 JSON 输出
"""

__all__ = [
    "types",
    "camera_model",
    "pose_io",
    "person_yolo_tracker",
    "footpoint",
    "ground_projection",
    "state_filter",
    "bev_canvas",
    "io_utils",
]
