#!/usr/bin/env python3
"""
生成 VGGT 项目测试与原理说明 Word 文档。

本脚本只使用 Python 标准库手写最小 OOXML 结构，不依赖 python-docx、
pandoc 或 LibreOffice。这样在当前 WSL 环境里也能稳定生成 .docx。
"""

from __future__ import annotations

import json
import zipfile
from datetime import datetime
from pathlib import Path
from xml.sax.saxutils import escape


REPO = Path("/home/ros/ros2_orbslam3")
OUTPUT_DOCX = REPO / "VGGT_测试与原理说明.docx"
VGGT_ROOT = REPO / "Opensource code" / "vggt-main" / "vggt-main"
VIDEO_PATH = REPO / "resources" / "input_video.mp4"
RESULT_DIR = REPO / "output" / "vggt_input_video_show"
FULL_VIEWER_DIR = RESULT_DIR / "full_viewer"
OFFICIAL_COLMAP_SUMMARY = RESULT_DIR / "official_colmap_summary.json"
GSPLAT_COMMAND = RESULT_DIR / "gsplat_command.txt"
GSPLAT_RESULTS_DIR = RESULT_DIR / "gsplat_results"
GSPLAT_RUN_DIR = RESULT_DIR / "gsplat_run"
GSPLAT_CFG = GSPLAT_RESULTS_DIR / "cfg.yml"
GSPLAT_STATS = GSPLAT_RESULTS_DIR / "stats" / "train_step29999_rank0.json"
GSPLAT_CKPT = GSPLAT_RESULTS_DIR / "ckpts" / "ckpt_29999_rank0.pt"
GSPLAT_PLY = GSPLAT_RESULTS_DIR / "ply" / "point_cloud_29999.ply"
GSPLAT_LOG = GSPLAT_RUN_DIR / "gsplat.log"
GSPLAT_DEBUG_DIR = GSPLAT_RESULTS_DIR / "debug_renders"
GSPLAT_DEBUG_FILTER_DIR = GSPLAT_RESULTS_DIR / "debug_renders_filter"
GSPLAT_DEBUG_SETTINGS_DIR = GSPLAT_RESULTS_DIR / "debug_renders_settings"


def read_json(path: Path) -> dict:
    """读取 JSON 文件；文件不存在时返回空字典，方便文档先生成后补测试。"""
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def read_text_if_exists(path: Path, max_chars: int = 4000) -> str:
    """读取普通文本；主要用于把命令或日志摘要写入文档。"""
    if not path.exists():
        return ""
    text = path.read_text(encoding="utf-8", errors="replace")
    if len(text) <= max_chars:
        return text
    return text[:max_chars] + "\n... 已截断，完整内容请看原文件。"


def text_run(text: str) -> str:
    """把普通文本转成 Word run XML。"""
    return f"<w:r><w:t xml:space=\"preserve\">{escape(text)}</w:t></w:r>"


def paragraph(text: str = "", style: str | None = None) -> str:
    """生成一个普通段落；style 可选 Heading1、Heading2、Code 等。"""
    style_xml = f"<w:pPr><w:pStyle w:val=\"{style}\"/></w:pPr>" if style else ""
    return f"<w:p>{style_xml}{text_run(text)}</w:p>"


def bullet(text: str) -> str:
    """用文本项目符号生成项目行，避免额外维护 numbering.xml。"""
    return paragraph(f"• {text}", "Normal")


def code_block(code: str) -> str:
    """把多行命令按等宽样式写入文档。"""
    return "".join(paragraph(line, "CodeBlock") for line in code.strip().splitlines())


def matrix_to_text(matrix) -> str:
    """把矩阵压成短文本，便于放进 Word。"""
    if not matrix:
        return "未生成"
    return "; ".join("[" + ", ".join(f"{float(v):.4f}" for v in row) + "]" for row in matrix)


def fmt(value, digits: int = 2) -> str:
    """把数字整理成便于文档阅读的文本。"""
    if value is None:
        return "未知"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def status_text(path: Path) -> str:
    """返回文件是否存在的短状态，方便在文档中做结果核对。"""
    return "已生成" if path.exists() else "未找到"


def build_document_body() -> str:
    """汇总固定说明和测试结果，生成 document.xml 的正文内容。"""
    summary = read_json(RESULT_DIR / "summary.json")
    frame_meta = read_json(RESULT_DIR / "frame_meta.json")
    full_meta = read_json(FULL_VIEWER_DIR / "vggt_full_windows_aggregated.json")
    official_summary = read_json(OFFICIAL_COLMAP_SUMMARY)
    gsplat_stats = read_json(GSPLAT_STATS)
    gsplat_command_text = read_text_if_exists(GSPLAT_COMMAND, max_chars=2500)
    generated_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    parts: list[str] = []
    parts.append(paragraph("VGGT 项目测试与原理说明", "Title"))
    parts.append(paragraph(f"适用于 {REPO} 工作区", "Subtitle"))
    parts.append(paragraph(f"生成时间：{generated_at}", "Subtitle"))
    parts.append(paragraph("说明：本文按照 DPVO 与 ORB-SLAM3 项目总文档的格式，整理当前工作区中 VGGT 的源码位置、运行链路、完整测试结果、3D 可视化方式、输出文件、核心原理和后续学习路线。", "Subtitle"))
    parts.append(paragraph())

    parts.append(paragraph("1. 项目定位", "Heading1"))
    parts.append(paragraph("VGGT 是 Visual Geometry Grounded Transformer 的缩写，可以理解为一种前馈式多视图几何模型。它的目标不是像 ORB-SLAM3 或 DPVO 那样持续在线跟踪，而是把一组图像输入到 Transformer 模型中，直接预测相机参数、深度图、三维点云和可选的点轨迹。"))
    parts.append(paragraph("在当前项目里，VGGT 的作用是给商场第一人称视频提供局部三维重建、深度估计和相机几何先验。它适合用来观察室内空间结构、评估遮挡和反光对深度的影响，并为后续 DPVO 主定位链路提供辅助参考。"))
    parts.append(paragraph("因此，VGGT 不是当前项目的唯一定位主线，而是和 DPVO、ORB-SLAM3 互补：ORB-SLAM3 负责传统 SLAM 基线，DPVO 负责连续视觉里程计，VGGT 负责多视图深度和点云重建。"))

    parts.append(paragraph("2. 总体框架", "Heading1"))
    parts.append(paragraph("从数据流上看，VGGT 这条链路不依赖 ROS2 话题，而是直接读取 resources 目录中的离线视频。项目包装脚本先抽取关键帧，再把小窗口图片送入 VGGT 官方模型，最后保存深度、置信度、相机参数和点云。"))
    parts.append(paragraph("昨天补齐的是后处理链路：先用官方 demo_colmap.py 把 VGGT 输出导成 COLMAP sparse 结构，并启用 BA 做几何优化；再用 gsplat 的 3D Gaussian Splatting 训练器把图像和 COLMAP 相机训练成可渲染的高斯场景；最后用 gsplat 官方 simple_viewer.py 打开实时 3DGS 查看器。"))
    parts.append(paragraph("3D 可视化分两层：scripts/open_vggt_viser.py 显示普通点云和图像面板；scripts/open_vggt_gsplat_viewer.sh 显示 3DGS 渲染结果。网页只是查看器，真正的结果都保存在 output 目录中。"))
    parts.append(code_block(f"""
商场视频 -> scripts/run_vggt_video.py -> 抽帧/滑动窗口 -> VGGT-1B 权重 -> 深度/置信度/相机/点云
VGGT 输出 -> scripts/open_vggt_viser.py -> Viser 服务器 -> 浏览器 3D 点云页面
VGGT 输出 -> scripts/run_vggt_official_colmap.py -> 官方 COLMAP/BA -> sparse/cameras.bin、images.bin、points3D.bin
COLMAP sparse + images -> scripts/run_vggt_gsplat_train.py -> gsplat/3DGS -> checkpoint + Gaussian PLY
3DGS checkpoint -> scripts/open_vggt_gsplat_viewer.sh -> 浏览器实时 3DGS 渲染页面
完整点云结果 -> {RESULT_DIR} -> full_viewer/vggt_full_windows_aggregated.ply
完整 3DGS 结果 -> {GSPLAT_RESULTS_DIR}
"""))

    parts.append(paragraph("3. 当前工作区的关键目录", "Heading1"))
    parts.append(paragraph("下面这些目录是当前 VGGT 部分最重要的入口。以后你要换视频、找输出、改脚本或查源码，基本都从这里开始。"))
    parts.append(bullet(f"VGGT 论文：{REPO / 'VGGT_CVPR25.pdf'}"))
    parts.append(bullet(f"VGGT 官方源码：{VGGT_ROOT}"))
    parts.append(bullet(f"输入视频目录：{REPO / 'resources'}"))
    parts.append(bullet(f"项目包装脚本目录：{REPO / 'scripts'}"))
    parts.append(bullet(f"可选本地依赖目录：{REPO / 'thirdparty' / 'python_packages'}"))
    parts.append(bullet(f"当前完整结果目录：{RESULT_DIR}"))
    parts.append(bullet(f"完整 3D 聚合结果目录：{FULL_VIEWER_DIR}"))
    parts.append(bullet(f"官方 COLMAP/BA 结果目录：{RESULT_DIR / 'sparse'}"))
    parts.append(bullet(f"3DGS 训练结果目录：{GSPLAT_RESULTS_DIR}"))
    parts.append(bullet(f"模型权重缓存目录：{REPO / '.cache' / 'torch' / 'hub' / 'checkpoints'}"))
    parts.append(paragraph("3.1 重点文件速查", "Heading2"))
    parts.append(bullet(f"运行视频推理：{REPO / 'scripts' / 'run_vggt_video.py'}"))
    parts.append(bullet(f"打开 3D 可视化：{REPO / 'scripts' / 'open_vggt_viser.py'}"))
    parts.append(bullet(f"官方 COLMAP/BA/3DGS 入口：{REPO / 'scripts' / 'run_vggt_official_colmap.py'}"))
    parts.append(bullet(f"运行 gsplat/3DGS 训练：{REPO / 'scripts' / 'run_vggt_gsplat_train.py'}"))
    parts.append(bullet(f"后台运行 3DGS 训练：{REPO / 'scripts' / 'run_vggt_gsplat_background.sh'}"))
    parts.append(bullet(f"打开 3DGS 渲染查看器：{REPO / 'scripts' / 'open_vggt_gsplat_viewer.sh'}"))
    parts.append(bullet(f"生成本文档：{REPO / 'scripts' / 'create_vggt_docx.py'}"))
    parts.append(bullet(f"官方 Viser 示例：{VGGT_ROOT / 'demo_viser.py'}"))
    parts.append(bullet(f"官方 Gradio 示例：{VGGT_ROOT / 'demo_gradio.py'}"))
    parts.append(bullet(f"官方 COLMAP 示例：{VGGT_ROOT / 'demo_colmap.py'}"))

    parts.append(paragraph("4. VGGT 的工作原理", "Heading1"))
    parts.append(paragraph("VGGT 的核心思想是：用一个大规模 Transformer 同时看多张图像，让模型在网络内部学习多视图几何关系，然后一次性输出相机、深度和三维结构。传统 SLAM 往往需要特征点、匹配、PnP、三角化和 Bundle Adjustment；VGGT 则把大量几何先验学习进模型权重里，推理时直接给出结果。"))
    parts.append(paragraph("4.1 模型模块", "Heading2"))
    parts.append(bullet("图像预处理：把 RGB 图像缩放、padding 或 crop 到模型需要的尺寸，并转成张量。"))
    parts.append(bullet("Patch Embedding：把图像切成 patch，并映射成 token。"))
    parts.append(bullet("Aggregator：在帧内和帧间做 attention，使模型同时理解单帧内容和跨帧几何关系。"))
    parts.append(bullet("Camera Head：预测相机位姿编码，再恢复外参 [R|t] 和内参 K。"))
    parts.append(bullet("Depth Head：输出每个像素的深度 depth 和深度置信度 depth_conf。"))
    parts.append(bullet("Point Head：可直接预测 world_points，但本项目默认关闭以节省 6GB 显存。"))
    parts.append(bullet("Track Head：可跟踪给定点在多帧中的位置，本项目默认关闭。"))
    parts.append(paragraph("4.2 关键公式", "Heading2"))
    formulas = [
        "Patch token：x_i = W_p * patch_i + b_p",
        "注意力机制：Attention(Q,K,V)=softmax(QK^T/sqrt(d))V",
        "内参矩阵：K=[[f_x,0,c_x],[0,f_y,c_y],[0,0,1]]",
        "视场角恢复焦距：f_x=(W/2)/tan(FoV_w/2)，f_y=(H/2)/tan(FoV_h/2)",
        "针孔投影：s[u,v,1]^T = K[R|t][X,Y,Z,1]^T",
        "深度反投影：X_c = Z K^{-1}[u,v,1]^T",
        "从相机坐标到世界坐标：X_w = R^T(X_c - t)",
        "相机中心：C = -R^T t",
        "点云筛选：保留 depth_conf 高于指定百分位的点，再写成 PLY。",
    ]
    for item in formulas:
        parts.append(bullet(item))

    parts.append(paragraph("5. 当前完整测试结果", "Heading1"))
    if summary:
        parts.append(bullet(f"输入视频：{summary.get('video', VIDEO_PATH)}"))
        parts.append(bullet(f"视频分辨率：{frame_meta.get('width', '未知')}x{frame_meta.get('height', '未知')}" if frame_meta else "视频分辨率：见 frame_meta.json"))
        parts.append(bullet(f"视频 FPS：{fmt(summary.get('video_fps'), 4)}"))
        parts.append(bullet(f"视频总帧数：{summary.get('video_total_frames', '未知')}"))
        parts.append(bullet(f"视频总时长：{fmt(summary.get('video_duration_sec'), 2)} 秒"))
        parts.append(bullet(f"抽帧间隔：每 {summary.get('frame_step', '未知')} 个原始帧取 1 帧"))
        parts.append(bullet(f"抽样帧数：{summary.get('processed_sample_frames', '未知')}"))
        parts.append(bullet(f"窗口设置：window-size={summary.get('window_size', '未知')}，window-stride={summary.get('window_stride', '未知')}"))
        parts.append(bullet(f"完成窗口数：{summary.get('num_windows', '未知')}"))
        parts.append(bullet(f"点云来源：{summary.get('point_cloud_source', '未知')}"))
        parts.append(bullet(f"总耗时：{fmt(summary.get('total_elapsed_sec'), 2)} 秒"))
        parts.append(bullet(f"预览视频：{summary.get('preview_video', RESULT_DIR / 'preview.mp4')}"))
    else:
        parts.append(bullet("尚未读取到 summary.json。请先运行 scripts/run_vggt_video.py 生成结果。"))
    if full_meta:
        parts.append(bullet(f"完整聚合窗口数：{full_meta.get('num_windows_used', '未知')}"))
        parts.append(bullet(f"每窗口保留点数：{full_meta.get('points_per_window', '未知')}"))
        parts.append(bullet(f"完整聚合点数：{full_meta.get('aggregated_points', '未知')}"))
        parts.append(bullet(f"完整聚合点云：{full_meta.get('aggregated_ply', FULL_VIEWER_DIR / 'vggt_full_windows_aggregated.ply')}"))
        parts.append(bullet(f"聚合布局：{full_meta.get('layout', 'unknown')}"))
        parts.append(bullet("重要说明：完整聚合点云是把多个局部窗口按时间展开后用于观察的结果，不等同于严格全局一致地图。"))

    parts.append(paragraph("6. 运行方法", "Heading1"))
    parts.append(paragraph("6.1 运行完整视频推理", "Heading2"))
    parts.append(paragraph("这条命令会读取 resources/input_video.mp4，按每 30 帧抽 1 帧的方式处理完整视频，并把结果保存到 output/vggt_input_video_show。"))
    parts.append(code_block(f"""
cd {REPO}
/home/ros/miniconda3/envs/dpvo/bin/python scripts/run_vggt_video.py \\
  --video resources/input_video.mp4 \\
  --frame-step 30 \\
  --window-size 2 \\
  --window-stride 1 \\
  --output-dir output/vggt_input_video_show
"""))
    parts.append(paragraph("6.2 运行时显示深度预览窗口", "Heading2"))
    parts.append(paragraph("如果 WSLg 图形显示正常，可以加 --show。这个窗口显示的是原图、深度图和置信度图，不是论文里那种 3D 点云页面。"))
    parts.append(code_block(f"""
cd {REPO}
/home/ros/miniconda3/envs/dpvo/bin/python scripts/run_vggt_video.py \\
  --video resources/input_video.mp4 \\
  --frame-step 30 \\
  --window-size 2 \\
  --window-stride 1 \\
  --show \\
  --output-dir output/vggt_input_video_show
"""))
    parts.append(paragraph("6.3 打开完整 3D 视图", "Heading2"))
    parts.append(paragraph("这条命令读取已生成的 windows/window_xxxx/vggt_points.ply，把所有窗口聚合成完整点云，并启动 Viser 网页查看器。"))
    parts.append(code_block(f"""
cd {REPO}
/home/ros/miniconda3/envs/dpvo/bin/python scripts/open_vggt_viser.py \\
  --result-dir output/vggt_input_video_show \\
  --aggregate-windows \\
  --points-per-window 4000 \\
  --max-points 350000 \\
  --layout timeline \\
  --port 8092
"""))
    parts.append(bullet("当前已启动的完整 3D 视图地址：http://localhost:8092"))
    parts.append(bullet("退出 viewer：回到运行命令的终端按 Ctrl+C。"))
    parts.append(paragraph("6.4 查看单个窗口的 3D 结果", "Heading2"))
    parts.append(paragraph("如果只想看最后一个窗口的原始预测结果，可以不加 --aggregate-windows。这个模式会显示 predictions.npz 中保存的点云和相机位姿。"))
    parts.append(code_block(f"""
cd {REPO}
/home/ros/miniconda3/envs/dpvo/bin/python scripts/open_vggt_viser.py \\
  --result-dir output/vggt_input_video_show \\
  --port 8090
"""))
    parts.append(paragraph("6.5 运行官方 COLMAP + BA 后处理", "Heading2"))
    parts.append(paragraph("这一步把 VGGT 已经抽好的 images/ 目录交给官方 demo_colmap.py，生成 COLMAP 标准 sparse 目录。当前为了适配 6GB 显存，使用 image-step=20、max-images=6 做抽样，并启用 --use-ba。"))
    parts.append(code_block(f"""
cd {REPO}
/home/ros/miniconda3/envs/dpvo/bin/python scripts/run_vggt_official_colmap.py \\
  --scene-dir output/vggt_input_video_show \\
  --use-ba \\
  --image-step 20 \\
  --max-images 6 \\
  --query-frame-num 2 \\
  --max-query-pts 256 \\
  --gsplat-root thirdparty/gsplat
"""))
    parts.append(bullet("输出 sparse 目录：output/vggt_input_video_show/sparse"))
    parts.append(bullet("官方抽样输入目录：output/vggt_input_video_show/_official_colmap_input_step20"))
    parts.append(bullet("参数记录文件：output/vggt_input_video_show/official_colmap_summary.json"))

    parts.append(paragraph("6.6 运行 3DGS / gsplat 训练", "Heading2"))
    parts.append(paragraph("这一步读取 output/vggt_input_video_show/images 和 sparse，把 COLMAP 相机与图像训练成 3D Gaussian Splatting 场景。训练时间较长，昨天完整训练 30000 步约 1 小时 37 分钟，建议放在 tmux 或后台脚本中跑。"))
    parts.append(code_block(f"""
cd {REPO}
/home/ros/miniconda3/envs/dpvo/bin/python scripts/run_vggt_gsplat_train.py \\
  --scene-dir output/vggt_input_video_show \\
  --result-dir output/vggt_input_video_show/gsplat_results \\
  --steps 30000 \\
  --tb-every 100
"""))
    parts.append(paragraph("如果远程终端容易断开，可以用后台脚本管理训练："))
    parts.append(code_block(f"""
cd {REPO}
scripts/run_vggt_gsplat_background.sh start
scripts/run_vggt_gsplat_background.sh status
scripts/run_vggt_gsplat_background.sh log
"""))
    parts.append(bullet(f"训练日志：{GSPLAT_LOG}"))
    parts.append(bullet(f"最终 checkpoint：{GSPLAT_CKPT}"))
    parts.append(bullet(f"导出的高斯 PLY：{GSPLAT_PLY}"))

    parts.append(paragraph("6.7 打开 3DGS 渲染查看器", "Heading2"))
    parts.append(paragraph("这一步读取 gsplat 的 checkpoint，不是读取普通点云 PLY。浏览器里看到的是高斯场景实时渲染效果。"))
    parts.append(code_block(f"""
cd {REPO}
scripts/open_vggt_gsplat_viewer.sh 8092
"""))
    parts.append(bullet("本地网址：http://127.0.0.1:8092"))
    parts.append(bullet("如果需要从 Windows 浏览器访问，也可以打开 http://localhost:8092"))
    parts.append(bullet("退出 viewer：回到运行命令的终端按 Ctrl+C。"))

    parts.append(paragraph("7. 输出文件说明", "Heading1"))
    parts.append(bullet(f"完整结果目录：{RESULT_DIR}"))
    parts.append(bullet("images/*.png：从输入视频中抽出来的关键帧图片。"))
    parts.append(bullet("preview.mp4：原图、深度图、置信度图拼接成的预览视频。"))
    parts.append(bullet("frame_meta.json：记录抽帧索引、原始帧号、时间戳和视频基础信息。"))
    parts.append(bullet("summary.json / summary.txt：记录完整运行参数、窗口数、耗时和输出位置。"))
    parts.append(bullet("camera_centers.txt：汇总各窗口中的相机中心估计，便于后续和 DPVO/ORB-SLAM3 轨迹做对照。"))
    parts.append(bullet("predictions.npz：默认保存最后一个窗口的完整数值结果，包括深度、相机内参、外参和点云数组。"))
    parts.append(bullet("vggt_points.ply：第一个窗口导出的点云，用于快速检查点云格式是否正常。"))
    parts.append(bullet("windows/window_xxxx/vggt_points.ply：每个窗口的局部点云。"))
    parts.append(bullet("windows/window_xxxx/previews/depth_000.png：每个窗口内单帧深度图。"))
    parts.append(bullet("windows/window_xxxx/previews/confidence_000.png：每个窗口内单帧深度置信度图。"))
    parts.append(bullet("full_viewer/vggt_full_windows_aggregated.ply：完整窗口聚合点云，是当前最重要的 3D 输出文件。"))
    parts.append(bullet("full_viewer/vggt_full_windows_aggregated.json：完整聚合点云的参数说明。"))
    parts.append(bullet("official_colmap_summary.json：官方 COLMAP/BA 后处理参数摘要。"))
    parts.append(bullet("gsplat_command.sh / gsplat_command.txt：官方 3DGS 训练命令。"))
    parts.append(bullet("sparse/cameras.bin：COLMAP 相机内参文件，是 3DGS 读取相机模型的入口。"))
    parts.append(bullet("sparse/images.bin：COLMAP 图像位姿文件，记录每张图片对应的相机外参。"))
    parts.append(bullet("sparse/points3D.bin：COLMAP 稀疏三维点文件，用于 3DGS 的 SfM 初始化。"))
    parts.append(bullet("gsplat_results/cfg.yml：3DGS 训练配置，记录步数、学习率、初始化方式和 refine 策略。"))
    parts.append(bullet("gsplat_results/ckpts/ckpt_29999_rank0.pt：训练 30000 步后保存的 3DGS checkpoint，是 viewer 的主要输入。"))
    parts.append(bullet("gsplat_results/ply/point_cloud_29999.ply：训练后导出的高斯点云文件，可用 CloudCompare 或 MeshLab 打开。"))
    parts.append(bullet("gsplat_results/stats/train_step29999_rank0.json：最后一步训练统计，包括显存、耗时和高斯数量。"))
    parts.append(bullet("gsplat_run/gsplat.log：完整训练日志，可用 tail 查看训练进度和结束状态。"))

    parts.append(paragraph("8. 调用原理：这些脚本到底在做什么", "Heading1"))
    parts.append(paragraph("scripts/run_vggt_video.py 是当前项目自己的包装层。它不是重写 VGGT，而是负责做工程适配：选择 conda 环境、读取视频、抽帧、构造滑动窗口、导入官方 VGGT 源码、加载权重、调用模型前向推理，然后把结果写成图片、视频、JSON、NPZ 和 PLY。"))
    parts.append(paragraph("scripts/open_vggt_viser.py 是结果查看层。它读取已经保存的点云或 predictions.npz，然后启动 Viser 本地网页服务。单窗口模式适合看模型原始输出；完整聚合模式适合看整段视频的窗口级点云序列。"))
    parts.append(paragraph("scripts/run_vggt_official_colmap.py 是官方后处理入口。它不改 VGGT 本体，而是把已经准备好的 images/ 目录交给官方 demo_colmap.py，输出 COLMAP 的 sparse/cameras.bin、images.bin、points3D.bin，然后再生成一条官方推荐的 gsplat 训练命令。"))
    parts.append(paragraph("这两层都不依赖 ROS2，所以运行 VGGT 时不用 source /opt/ros/humble/setup.bash。当前之所以复用 dpvo conda 环境，是因为里面已经有 CUDA 版 PyTorch 和一批视觉依赖，VGGT 和 DPVO 没有算法依赖关系，只是工程上共享同一个 Python 环境。"))

    parts.append(paragraph("8.1 官方 COLMAP + BA + 3DGS 路线", "Heading2"))
    parts.append(paragraph("VGGT 官方 README 明确支持两条后处理路径：第一条是直接导出 COLMAP 格式；第二条是开启 --use_ba 后，先用 tracker 生成轨迹，再用 pycolmap 做 Bundle Adjustment。官方还说明导出的 COLMAP 结果可以直接接 gsplat 做 Gaussian Splatting。"))
    parts.append(bullet("非 BA 路线：速度快，主要把 VGGT 的深度和位姿结果导出到 COLMAP sparse 文件。"))
    parts.append(bullet("BA 路线：会额外做轨迹预测和 pycolmap 优化，几何一致性通常更好，但依赖更多、显存和时间开销也更高。"))
    parts.append(bullet("3DGS 路线：使用官方导出的 COLMAP 结果作为输入，把多视图图像训练成可渲染的高斯场景。"))
    parts.append(bullet("当前工作区新增了 scripts/run_vggt_official_colmap.py，负责把这条官方路线接到现有 output/vggt_input_video_show 目录。"))
    parts.append(bullet("当前状态：官方 COLMAP/BA 已跑通，sparse 目录已生成；gsplat 3DGS 已完成 30000 步训练；viewer 已切到官方 nerfview/gsplat 查看器。"))
    parts.append(bullet("这条路线最接近论文里看到的 3D 场景展示，但最终效果仍强依赖视频质量、相机位姿一致性、动态行人、反光地面和纹理重复情况。"))

    parts.append(paragraph("9. GPU、训练集和显存", "Heading1"))
    parts.append(paragraph("当前运行 VGGT 是推理，不是训练。推理时会加载预训练权重 model.pt，然后用 GPU 执行 Transformer、深度预测头和相机预测头。RTX 4050 Laptop GPU 的 6GB 显存可以跑小窗口，但不适合一次性把完整长视频塞进模型。"))
    parts.append(bullet("GPU 主要参与：图像张量计算、Transformer attention、深度预测、相机预测和张量后处理。"))
    parts.append(bullet("GPU 还参与：gsplat 的 CUDA 扩展编译、3DGS 前向渲染、反向传播、Gaussian 参数优化和 viewer 中的实时渲染。"))
    parts.append(bullet("CPU 主要参与：视频解码、抽帧、文件读写、JSON/PLY 保存、COLMAP 数据结构管理和 Viser/nerfview 服务启动。"))
    parts.append(bullet("训练集的作用：在模型训练阶段提供多视图图像、相机、深度或几何监督，让网络学习通用三维先验。"))
    parts.append(bullet("当前推理阶段不使用训练集，只使用输入视频和预训练权重。"))
    parts.append(bullet("3DGS 阶段不是训练 VGGT 神经网络，而是在当前视频场景上优化一组 3D 高斯参数，所以它属于场景重建优化，不属于 VGGT 模型微调。"))
    parts.append(bullet("如果效果不好，优先改抽帧、窗口大小、遮挡处理和视频质量；最后才考虑微调训练。"))
    parts.append(bullet("官方 BA 路线还会再调用 keypoint extractor、tracker 和 pycolmap 的优化模块，因此比纯前向推理更吃资源。"))
    parts.append(bullet("当前 3DGS 完整训练统计：30000 步，最终高斯数量约 1586784，训练端统计显存约 2.49GB，耗时约 5851.60 秒。"))

    parts.append(paragraph("10. VGGT、DPVO、ORB-SLAM3 的区别", "Heading1"))
    parts.append(bullet("ORB-SLAM3：传统特征点 SLAM，强调实时跟踪、建图、回环和重定位，主要依赖 CPU；单目尺度天然不确定。"))
    parts.append(bullet("DPVO：深度学习视觉里程计，面向连续视频轨迹估计，依赖 GPU 和 CUDA 扩展，是当前项目后续定位主线。"))
    parts.append(bullet("VGGT：前馈式多视图几何模型，适合抽取关键帧后做深度估计、局部点云和相机先验，不是完整在线 SLAM 系统。"))
    parts.append(bullet("三者关系：ORB-SLAM3 做传统基线，DPVO 做连续定位，VGGT 做局部几何理解和重建辅助。"))

    parts.append(paragraph("11. 如果要更换输入视频", "Heading1"))
    parts.append(paragraph("VGGT 换视频比 ROS2 节点简单，通常只需要改 --video 参数和 --output-dir。建议每个输入视频使用独立输出目录，避免结果互相覆盖。"))
    parts.append(code_block(f"""
cd {REPO}
/home/ros/miniconda3/envs/dpvo/bin/python scripts/run_vggt_video.py \\
  --video resources/你的新视频.mp4 \\
  --frame-step 30 \\
  --window-size 2 \\
  --window-stride 1 \\
  --output-dir output/vggt_你的新视频名
"""))
    parts.append(bullet("如果视频很长，先用 --duration-sec 10 做短测试。"))
    parts.append(bullet("如果显存不够，降低 --window-size，或者增大 --frame-step。"))
    parts.append(bullet("如果想保留每个窗口的完整数值预测，加 --save-window-npz，但磁盘占用会明显增加。"))
    parts.append(bullet("如果使用遮挡后视频，例如 input_video.mp4_bev.mp4，直接把 --video 换成对应路径即可。"))

    parts.append(paragraph("12. 常见问题", "Heading1"))
    parts.append(paragraph("问题 1：为什么只能看到深度图视频，看不到论文里那种酷炫 3D 效果？"))
    parts.append(paragraph("答：preview.mp4 只是二维预览。论文里的效果来自点云、相机位姿和交互式 3D viewer。当前要看 3D 效果，需要运行 scripts/open_vggt_viser.py 并打开 http://localhost:8092。"))
    parts.append(paragraph("问题 2：为什么完整聚合点云不是严格地图？"))
    parts.append(paragraph("答：当前脚本把每个窗口的局部点云居中后按时间展开，目的是快速观察整段视频结构。VGGT 分窗口输出本身没有做全局 BA、回环或尺度统一，所以它不是严格 SLAM 地图。"))
    parts.append(paragraph("问题 3：为什么复用 dpvo conda 环境？"))
    parts.append(paragraph("答：这是工程复用，不是算法依赖。dpvo 环境里已经有 CUDA 版 PyTorch，能直接跑 VGGT；VGGT 和 DPVO 的网络结构、权重和算法目标是分开的。"))
    parts.append(paragraph("问题 4：终端里出现 libtinfo.so.6 no version information 是失败吗？"))
    parts.append(paragraph("答：通常不是。这是 conda 动态库和系统 bash 的兼容提示，只要后续程序继续运行、summary 和 PLY 正常生成，就可以暂时忽略。"))
    parts.append(paragraph("问题 5：为什么打开 3DGS viewer 一开始是空白？"))
    parts.append(paragraph("答：之前的原因是 Python 先导入了 thirdparty/python_packages 里的 nerfview 兼容壳。那个兼容壳只能让 import 不报错，但 rerender 实际不会把 3DGS 渲染结果推到网页。现在 scripts/open_vggt_gsplat_viewer.sh 已经改成不加入 thirdparty/python_packages，并使用 conda 环境里的官方 nerfview。"))
    parts.append(paragraph("问题 6：为什么能打开 3DGS viewer，但看到的是杂乱毛刺？"))
    parts.append(paragraph("答：这说明后处理链路已经跑通，但输入视频对应的几何和位姿不够稳定。商场第一人称视频里有动态行人、反光地面、运动模糊、重复纹理和大转弯，这些会让 VGGT/COLMAP/BA 得到的相机和稀疏点不一致，3DGS 会被错误几何带偏。viewer 参数只能减少视觉杂点，不能从根本上修复坏几何。"))
    parts.append(paragraph("问题 7：为什么普通 PLY 只有一堆点，没有论文里的完整图像场景？"))
    parts.append(paragraph("答：普通 PLY 是点云；论文风格的视图通常是点云、相机图像面板、COLMAP/BA 和 3D Gaussian Splatting 一起展示。当前 full_viewer/vggt_full_windows_aggregated.ply 只是快速检查点云，真正接近论文风格的是 gsplat_results/ckpts/ckpt_29999_rank0.pt 通过 open_vggt_gsplat_viewer.sh 打开的 3DGS viewer。"))
    parts.append(paragraph("问题 8：3DGS 结果差，是不是训练没跑完？"))
    parts.append(paragraph("答：不是。当前日志显示训练已经完成 30000/30000 步，并保存了 checkpoint 和 PLY。质量差主要来自数据与几何一致性，而不是训练中断。"))

    parts.append(paragraph("13. 后续学习路线", "Heading1"))
    parts.append(paragraph("如果你想把 VGGT 真正用到项目里，建议按下面顺序学习和扩展。"))
    parts.append(bullet("第一步：掌握相机内参 K、外参 [R|t]、深度图、反投影和坐标系转换。"))
    parts.append(bullet("第二步：读懂 scripts/run_vggt_video.py，理解抽帧、窗口、模型推理和文件输出。"))
    parts.append(bullet("第三步：读懂 scripts/open_vggt_viser.py，理解 PLY 点云、Viser 服务和网页查看器之间的关系。"))
    parts.append(bullet("第四步：跑官方 COLMAP 导出，观察 sparse/cameras.bin、images.bin、points3D.bin 是怎么生成的。"))
    parts.append(bullet("第五步：如果需要更高几何一致性，再尝试 --use_ba，理解 tracker 和 pycolmap 优化在做什么。"))
    parts.append(bullet("第六步：把导出的 COLMAP 结果接到 gsplat，理解 3DGS 的训练和渲染流程。"))
    parts.append(bullet("第七步：对比同一段视频下的 ORB-SLAM3、DPVO 和 VGGT 输出，观察哪一段稳定、哪一段失败。"))
    parts.append(bullet("第八步：如果未来显存和数据充足，再考虑使用室内商场数据做微调。"))

    parts.append(paragraph("附：推荐的日常使用顺序", "Heading2"))
    parts.append(code_block(f"""
1) 先跑短片段冒烟测试：scripts/run_vggt_video.py --duration-sec 10
2) 再跑完整视频：scripts/run_vggt_video.py --output-dir output/vggt_input_video_show
3) 看二维深度预览：打开 output/vggt_input_video_show/preview.mp4
4) 看完整 3D 点云：打开 http://localhost:8092
5) 跑官方 COLMAP：scripts/run_vggt_official_colmap.py --scene-dir output/vggt_input_video_show
6) 再接 3DGS：scripts/run_vggt_gsplat_train.py --steps 30000
7) 打开 3DGS：scripts/open_vggt_gsplat_viewer.sh 8092
8) 保存关键结果：full_viewer/vggt_full_windows_aggregated.ply、sparse/、gsplat_results/ 和 summary.json
"""))

    if official_summary:
        parts.append(paragraph("14. 官方 COLMAP / BA / 3DGS 结果", "Heading1"))
        parts.append(bullet(f"官方结果目录：{official_summary.get('sparse_dir', RESULT_DIR / 'sparse')}"))
        parts.append(bullet(f"官方抽样输入目录：{official_summary.get('official_scene_dir', RESULT_DIR / '_official_colmap_input_step20')}"))
        parts.append(bullet(f"官方 sparse 原始目录：{official_summary.get('official_sparse_dir', RESULT_DIR / '_official_colmap_input_step20' / 'sparse')}"))
        parts.append(bullet(f"是否启用 BA：{official_summary.get('use_ba', False)}"))
        parts.append(bullet(f"相机模型：{official_summary.get('camera_type', 'SIMPLE_PINHOLE')}"))
        parts.append(bullet(f"最大重投影误差：{official_summary.get('max_reproj_error', '未知')}"))
        parts.append(bullet(f"抽样步长 image-step：{official_summary.get('image_step', '未知')}"))
        parts.append(bullet(f"最多图片数 max-images：{official_summary.get('max_images', '未知')}"))
        parts.append(bullet(f"query-frame-num：{official_summary.get('query_frame_num', '未知')}"))
        parts.append(bullet(f"max-query-pts：{official_summary.get('max_query_pts', '未知')}"))
        parts.append(bullet(f"官方 demo_colmap 命令：{' '.join(official_summary.get('demo_colmap_command', []))}"))
        parts.append(bullet(f"3DGS 命令文件：{GSPLAT_COMMAND}"))
        parts.append(bullet(f"sparse/cameras.bin：{status_text(RESULT_DIR / 'sparse' / 'cameras.bin')}"))
        parts.append(bullet(f"sparse/images.bin：{status_text(RESULT_DIR / 'sparse' / 'images.bin')}"))
        parts.append(bullet(f"sparse/points3D.bin：{status_text(RESULT_DIR / 'sparse' / 'points3D.bin')}"))
        parts.append(bullet("说明：当前已经完成官方 COLMAP/BA 导出，sparse 目录可直接作为 3DGS 输入。"))

    parts.append(paragraph("15. 昨天完成的 VGGT 后处理完整流程", "Heading1"))
    parts.append(paragraph("这一节记录已经实际跑过的完整流程，方便以后从头复现。它不是只写理论，而是对应当前工作区真实存在的脚本和输出文件。"))
    parts.append(paragraph("15.1 第一步：VGGT 完整视频推理", "Heading2"))
    parts.append(paragraph("输入是 resources/input_video.mp4。脚本按每 30 帧抽 1 帧，共抽到 107 张图；每个窗口 2 张图，步长 1，所以得到 106 个局部窗口。每个窗口生成深度图、置信度图、局部点云和相机中心。"))
    parts.append(bullet(f"结果目录：{RESULT_DIR}"))
    parts.append(bullet(f"二维预览视频：{RESULT_DIR / 'preview.mp4'}"))
    parts.append(bullet(f"抽帧图像目录：{RESULT_DIR / 'images'}"))
    parts.append(bullet(f"局部窗口目录：{RESULT_DIR / 'windows'}"))
    parts.append(bullet(f"完整视频 VGGT 耗时：{fmt(summary.get('total_elapsed_sec') if summary else None, 2)} 秒"))

    parts.append(paragraph("15.2 第二步：点云聚合 viewer", "Heading2"))
    parts.append(paragraph("open_vggt_viser.py 把 106 个窗口的局部 PLY 聚合到 full_viewer 目录。这个结果适合快速看全视频的几何趋势，但它不是严格全局地图，因为每个 VGGT 窗口本来就在自己的局部坐标里。"))
    parts.append(bullet(f"聚合点云：{FULL_VIEWER_DIR / 'vggt_full_windows_aggregated.ply'}"))
    parts.append(bullet(f"聚合参数：{FULL_VIEWER_DIR / 'vggt_full_windows_aggregated.json'}"))
    parts.append(bullet(f"聚合窗口数：{full_meta.get('num_windows_used', '未知') if full_meta else '未知'}"))
    parts.append(bullet(f"聚合点数：{full_meta.get('aggregated_points', '未知') if full_meta else '未知'}"))

    parts.append(paragraph("15.3 第三步：官方 COLMAP + BA", "Heading2"))
    parts.append(paragraph("run_vggt_official_colmap.py 调用 VGGT 官方 demo_colmap.py。为了让 RTX 4050 的 6GB 显存稳定跑完，当前只从完整抽帧序列里每 20 张取 1 张，最多取 6 张，然后启用 --use_ba 做 Bundle Adjustment。"))
    parts.append(code_block("""
Bundle Adjustment 目标函数：
min_{R_i,t_i,K_j,X_k} Σ ρ( || π(K_j, R_i, t_i, X_k) - u_{ik} ||^2 )

含义：
R_i,t_i 是第 i 张图的相机外参；
K_j 是相机内参；
X_k 是三维点；
u_{ik} 是观测到的二维点；
π 是针孔投影函数；
ρ 是鲁棒损失，用来降低错误匹配的影响。
"""))
    parts.append(bullet("BA 的作用：让相机位姿、内参和三维点在多视角投影上尽量一致。"))
    parts.append(bullet("当前输出：output/vggt_input_video_show/sparse/cameras.bin、images.bin、points3D.bin。"))

    parts.append(paragraph("15.4 第四步：gsplat / 3D Gaussian Splatting", "Heading2"))
    parts.append(paragraph("run_vggt_gsplat_train.py 读取 images/ 和 sparse/，调用 thirdparty/gsplat/examples/simple_trainer.py。它不是训练 VGGT 模型，而是在当前视频场景上优化一组 3D 高斯，使这些高斯从训练相机角度渲染出来时尽量接近原图。"))
    parts.append(code_block("""
单个 3D Gaussian 的核心参数：
G = { μ, Σ, α, c }

μ：高斯中心位置；
Σ：三维协方差，决定高斯的大小、方向和形状；
α：透明度；
c：颜色，通常用球谐 SH 系数表示不同视角下的颜色。

渲染时把 3D 高斯投影到图像平面，按深度顺序做 alpha compositing：
C = Σ_i T_i α_i c_i
T_i = Π_{j<i}(1 - α_j)
"""))
    parts.append(bullet("训练目标：让渲染图像和真实输入图像的 L1/SSIM 损失尽量小。"))
    parts.append(bullet("初始化方式：init_type=sfm，使用 COLMAP sparse 点初始化高斯位置。"))
    parts.append(bullet("训练步数：30000。"))
    parts.append(bullet("SH 阶数：sh_degree=3。"))
    parts.append(bullet("保存 PLY：save_ply=true，ply_steps=[30000]。"))
    parts.append(bullet("最终 checkpoint：gsplat_results/ckpts/ckpt_29999_rank0.pt。"))
    parts.append(bullet("最终高斯 PLY：gsplat_results/ply/point_cloud_29999.ply。"))

    parts.append(paragraph("16. 3DGS 编译与 viewer 修复记录", "Heading1"))
    parts.append(paragraph("昨天遇到的关键问题有两个：一个是 gsplat CUDA 扩展编译时涉及 cuda/std/optional 头文件；另一个是 viewer 虽然打开网页但不渲染。"))
    parts.append(paragraph("16.1 CUDA 扩展编译修复", "Heading2"))
    parts.append(bullet("当前只需要标准 3DGS，因此 run_vggt_gsplat_train.py 强制 BUILD_3DGS=1，并关闭 BUILD_2DGS、BUILD_3DGUT、BUILD_ADAM、BUILD_RELOC、BUILD_CAMERA_WRAPPERS。"))
    parts.append(bullet("这样可以避开当前项目不需要的 3DGUT 分支，也绕开 cuda/std/optional 在 CUDA 12.1 环境中的兼容问题。"))
    parts.append(bullet("本地还保留了 thirdparty/cuda_cccl_shim/include/cuda/std/optional 作为兼容 shim。"))
    parts.append(bullet("thirdparty/gsplat/gsplat/cuda/build.py 已修正 -idirafter 参数形式，让额外 include 路径能被 CUDA 编译器正确识别。"))
    parts.append(paragraph("16.2 冒烟测试", "Heading2"))
    parts.append(bullet(f"3DGS-only 冒烟结果：{RESULT_DIR / 'gsplat_smoke_results_3dgs_only'}"))
    parts.append(bullet(f"背景修复后冒烟结果：{RESULT_DIR / 'gsplat_smoke_results_bg_after_fix'}"))
    parts.append(bullet("冒烟测试说明：用很少训练步数确认 CUDA 扩展能编译、训练入口能跑、结果目录能生成，然后再跑 30000 步完整版。"))
    parts.append(paragraph("16.3 viewer 修复", "Heading2"))
    parts.append(bullet("之前网页空白的原因不是 checkpoint 不存在，而是 Python 优先导入了 thirdparty/python_packages 中的 nerfview 兼容壳。"))
    parts.append(bullet("这个兼容壳的 rerender 是空操作，所以网页能开，但不会显示真实渲染。"))
    parts.append(bullet("已安装官方 nerfview 0.1.2，以及 splines、jaxtyping。"))
    parts.append(bullet("scripts/open_vggt_gsplat_viewer.sh 已改为只加入 gsplat 源码和 examples，不再加入 thirdparty/python_packages。"))

    parts.append(paragraph("17. 当前 3DGS 完整训练结果", "Heading1"))
    parts.append(bullet(f"训练日志：{GSPLAT_LOG}"))
    parts.append(bullet(f"训练配置：{GSPLAT_CFG}"))
    parts.append(bullet(f"最后统计：{GSPLAT_STATS}"))
    parts.append(bullet(f"checkpoint：{GSPLAT_CKPT}，状态：{status_text(GSPLAT_CKPT)}"))
    parts.append(bullet(f"高斯 PLY：{GSPLAT_PLY}，状态：{status_text(GSPLAT_PLY)}"))
    if gsplat_stats:
        parts.append(bullet(f"最终高斯数量 num_GS：{gsplat_stats.get('num_GS', '未知')}"))
        parts.append(bullet(f"训练端统计显存 mem：{fmt(gsplat_stats.get('mem'), 2)} GB"))
        parts.append(bullet(f"训练端统计耗时 ellipse_time：{fmt(gsplat_stats.get('ellipse_time'), 2)} 秒"))
    parts.append(bullet(f"调试渲染 front.png：{GSPLAT_DEBUG_DIR / 'front.png'}，状态：{status_text(GSPLAT_DEBUG_DIR / 'front.png')}"))
    parts.append(bullet(f"过滤调试图：{GSPLAT_DEBUG_FILTER_DIR / 'opa05_scale04_crop995.png'}，状态：{status_text(GSPLAT_DEBUG_FILTER_DIR / 'opa05_scale04_crop995.png')}"))
    parts.append(bullet(f"参数调试图：{GSPLAT_DEBUG_SETTINGS_DIR / 'sh0_r2_eps08_white.png'}，状态：{status_text(GSPLAT_DEBUG_SETTINGS_DIR / 'sh0_r2_eps08_white.png')}"))

    parts.append(paragraph("18. 当前效果评估与改进路线", "Heading1"))
    parts.append(paragraph("当前结论要分开看：技术链路已经跑通，但视觉质量没有达到论文演示里那种干净、连续、完整的场景效果。你在 viewer 里看到的杂乱毛刺，和离线调试渲染一致，所以问题主要不在浏览器，而在数据和几何一致性。"))
    parts.append(bullet("动态行人：人在画面中移动，会让同一空间位置在不同帧里不一致。"))
    parts.append(bullet("地面反光：反光不是稳定三维结构，会让深度和匹配产生错误。"))
    parts.append(bullet("第一人称运动：转弯、抖动和运动模糊会降低关键点与几何估计质量。"))
    parts.append(bullet("重复纹理：商场地砖、灯带、货架容易造成错误匹配。"))
    parts.append(bullet("抽样帧太少：为了显存只给 BA 用了 6 张图，几何约束偏少，场景连续性不够。"))
    parts.append(bullet("单目尺度问题：来自单目视频的深度和尺度没有真实传感器约束，长序列更容易漂移。"))
    parts.append(paragraph("建议的改进顺序如下："))
    parts.append(bullet("先截取 10 到 20 秒稳定直行、行人少、反光少的片段，单独跑一版 VGGT + COLMAP/BA + 3DGS，确认上限效果。"))
    parts.append(bullet("对输入视频做更强的动态物体遮挡，把行人、玻璃反光和大面积地面反射区域尽量排除。"))
    parts.append(bullet("在显存允许时提高 --max-images 或降低 --image-step，让 BA 使用更多视角。"))
    parts.append(bullet("优先选择转弯小、重叠区域多、画面清晰的关键帧。"))
    parts.append(bullet("先检查 COLMAP sparse 是否干净，再训练 3DGS；如果 sparse 已经乱，3DGS 基本也会乱。"))
    parts.append(bullet("如果目标是最终定位，DPVO 仍应作为主线；VGGT/3DGS 更适合作为三维理解和展示辅助。"))

    parts.append(paragraph("19. 关键代码入口说明", "Heading1"))
    parts.append(bullet("scripts/run_vggt_video.py：读取视频、抽帧、加载 VGGT、按窗口推理、保存深度/置信度/点云/summary。"))
    parts.append(bullet("scripts/open_vggt_viser.py：读取 VGGT 结果，用 Viser 显示普通点云、相机位置和图像面板。"))
    parts.append(bullet("scripts/run_vggt_official_colmap.py：封装官方 demo_colmap.py，生成 COLMAP sparse，并写出 3DGS 命令。"))
    parts.append(bullet("scripts/run_vggt_gsplat_train.py：设置 gsplat 所需 PYTHONPATH、CUDA 编译变量和训练参数，再调用 simple_trainer.py。"))
    parts.append(bullet("scripts/run_vggt_gsplat_background.sh：用后台方式启动、查看、停止 3DGS 训练，适合远程终端不稳定时使用。"))
    parts.append(bullet("scripts/open_vggt_gsplat_viewer.sh：读取 ckpt_29999_rank0.pt，启动官方 simple_viewer.py 查看 3DGS 场景。"))
    parts.append(bullet("thirdparty/gsplat/examples/simple_trainer.py：官方 gsplat 训练器，核心 3DGS 优化逻辑在这里。"))
    parts.append(bullet("thirdparty/gsplat/examples/simple_viewer.py：官方 gsplat 查看器，核心实时渲染入口在这里。"))

    if gsplat_command_text:
        parts.append(paragraph("附：当前记录的 3DGS 命令文件内容", "Heading2"))
        parts.append(code_block(gsplat_command_text))

    body = "".join(parts)
    return f"<w:body>{body}<w:sectPr><w:pgSz w:w=\"11906\" w:h=\"16838\"/><w:pgMar w:top=\"1440\" w:right=\"1440\" w:bottom=\"1440\" w:left=\"1440\" w:header=\"708\" w:footer=\"708\" w:gutter=\"0\"/></w:sectPr></w:body>"


def styles_xml() -> str:
    """定义 Word 文档中用到的基础样式。"""
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:style w:type="paragraph" w:default="1" w:styleId="Normal">
    <w:name w:val="Normal"/>
    <w:rPr><w:rFonts w:ascii="Microsoft YaHei" w:eastAsia="Microsoft YaHei"/><w:sz w:val="22"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Title">
    <w:name w:val="Title"/>
    <w:basedOn w:val="Normal"/>
    <w:rPr><w:b/><w:sz w:val="36"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Subtitle">
    <w:name w:val="Subtitle"/>
    <w:basedOn w:val="Normal"/>
    <w:rPr><w:sz w:val="22"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Heading1">
    <w:name w:val="Heading 1"/>
    <w:basedOn w:val="Normal"/>
    <w:rPr><w:b/><w:sz w:val="30"/></w:rPr>
    <w:pPr><w:spacing w:before="240" w:after="120"/></w:pPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Heading2">
    <w:name w:val="Heading 2"/>
    <w:basedOn w:val="Normal"/>
    <w:rPr><w:b/><w:sz w:val="25"/></w:rPr>
    <w:pPr><w:spacing w:before="160" w:after="80"/></w:pPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="CodeBlock">
    <w:name w:val="CodeBlock"/>
    <w:basedOn w:val="Normal"/>
    <w:rPr><w:rFonts w:ascii="Consolas" w:eastAsia="Microsoft YaHei"/><w:sz w:val="20"/></w:rPr>
  </w:style>
</w:styles>"""


def document_xml() -> str:
    """生成 Word 主文档 XML。"""
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
{build_document_body()}
</w:document>"""


def write_docx(path: Path) -> None:
    """把必要的 OOXML 文件打包成 .docx。"""
    content_types = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
  <Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
  <Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
  <Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
</Types>"""
    rels = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
</Relationships>"""
    document_rels = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"/>"""
    core = f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:dcterms="http://purl.org/dc/terms/" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <dc:title>VGGT 项目测试与原理说明</dc:title>
  <dc:creator>Codex</dc:creator>
  <cp:lastModifiedBy>Codex</cp:lastModifiedBy>
  <dcterms:created xsi:type="dcterms:W3CDTF">{datetime.utcnow().isoformat()}Z</dcterms:created>
  <dcterms:modified xsi:type="dcterms:W3CDTF">{datetime.utcnow().isoformat()}Z</dcterms:modified>
</cp:coreProperties>"""
    app = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties">
  <Application>Codex</Application>
</Properties>"""

    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as docx:
        docx.writestr("[Content_Types].xml", content_types)
        docx.writestr("_rels/.rels", rels)
        docx.writestr("word/document.xml", document_xml())
        docx.writestr("word/styles.xml", styles_xml())
        docx.writestr("word/_rels/document.xml.rels", document_rels)
        docx.writestr("docProps/core.xml", core)
        docx.writestr("docProps/app.xml", app)


def main() -> None:
    """脚本入口。"""
    write_docx(OUTPUT_DOCX)
    print(f"已生成 Word 文档: {OUTPUT_DOCX}")


if __name__ == "__main__":
    main()
