"""配置加载、视频读写、JSON 序列化的小工具集合。"""

from __future__ import annotations

import json
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import numpy as np


def load_config(path: str) -> Dict[str, Any]:
    """加载 YAML 配置。优先用 PyYAML，找不到时用极简解析。"""
    text = Path(path).read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore
    except Exception:
        return _simple_yaml(text)
    return yaml.safe_load(text)


def _simple_yaml(text: str) -> Dict[str, Any]:
    """极简 YAML 解析: 仅支持本项目使用的 2 层结构。"""
    import re

    root: Dict[str, Any] = {}
    current_key: Optional[str] = None
    for raw in text.splitlines():
        line = raw.rstrip()
        if not line or line.lstrip().startswith("#"):
            continue
        if not line.startswith(" "):
            m = re.match(r"^([\w\-]+):\s*(.*)$", line)
            if not m:
                continue
            k, v = m.group(1), m.group(2).strip()
            if v == "":
                root[k] = {}
                current_key = k
            else:
                root[k] = _parse_value(v)
                current_key = None
        else:
            if current_key is None:
                continue
            m = re.match(r"^\s+([\w\-]+):\s*(.*)$", line)
            if not m:
                continue
            k, v = m.group(1), m.group(2).strip()
            root[current_key][k] = _parse_value(v)
    return root


def _parse_value(v: str) -> Any:
    if v.startswith("[") and v.endswith("]"):
        body = v[1:-1].strip()
        if not body:
            return []
        parts = [p.strip().strip('"').strip("'") for p in body.split(",")]
        out = []
        for p in parts:
            try:
                out.append(int(p) if p.lstrip("-").isdigit() else float(p))
            except ValueError:
                out.append(p)
        return out
    if v.startswith('"') and v.endswith('"'):
        return v[1:-1]
    if v.lower() in ("true", "false"):
        return v.lower() == "true"
    try:
        if "." in v or "e" in v.lower():
            return float(v)
        return int(v)
    except ValueError:
        return v


def to_jsonable(obj: Any) -> Any:
    """递归把 numpy / dataclass 转成 JSON 可序列化结构。"""
    if obj is None:
        return None
    if isinstance(obj, (str, int, float, bool)):
        return obj
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, dict):
        return {str(k): to_jsonable(v) for k, v in obj.items()}
    if is_dataclass(obj):
        return to_jsonable(asdict(obj))
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(v) for v in obj]
    return str(obj)


def write_json(path: str, payload: Any) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(to_jsonable(payload), f, ensure_ascii=False, indent=2)


def open_video_reader(path: str):
    import cv2

    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise FileNotFoundError(f"cannot open video: {path}")
    return cap


def open_video_writer(path: str, fps: float, size: tuple, fourcc: str = "mp4v"):
    import cv2

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fourcc_code = cv2.VideoWriter_fourcc(*fourcc)
    writer = cv2.VideoWriter(path, fourcc_code, float(fps), tuple(int(x) for x in size))
    if not writer.isOpened():
        raise RuntimeError(f"cannot open writer: {path}")
    return writer


def iterate_frames(cap, max_frames: int = 0, frame_stride: int = 1) -> Iterable:
    import cv2  # noqa: F401

    idx = 0
    out_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if idx % max(1, frame_stride) == 0:
            yield idx, frame
            out_idx += 1
            if max_frames > 0 and out_idx >= max_frames:
                break
        idx += 1
