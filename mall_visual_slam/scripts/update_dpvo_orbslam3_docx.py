#!/usr/bin/env python3
"""把 DPVO 增强功能记录追加到 DPVO_ORBSLAM3_项目总文档.docx。

当前项目总文档是 Word 文件。这个脚本直接修改 docx 内部的
word/document.xml，在保留原有内容和样式的基础上追加一章中文说明。
"""

from __future__ import annotations

import shutil
import tempfile
import zipfile
from datetime import datetime
from pathlib import Path
from xml.sax.saxutils import escape


REPO = Path("/home/ros/ros2_orbslam3")
DOCX = REPO / "DPVO_ORBSLAM3_项目总文档.docx"
BACKUP = REPO / "DPVO_ORBSLAM3_项目总文档.before_dpvo_enhance.docx"
SECTION_MARKER = "14. DPVO 短期增强：关键帧记忆与跟踪质量评估"
SECOND_SECTION_MARKER = "15. DPVO 第二版增强：几何重定位验证"
THIRD_SECTION_MARKER = "16. DPVO 第三版增强：实时可视化、结果视频与 KV-Track3r 启发"
FOURTH_SECTION_MARKER = "17. DPVO 三维增强可视化：轨迹、相机视锥与点云接口"
FIFTH_SECTION_MARKER = "18. DPVO 三维显示坐标对齐与作者 demo 风格升级路线"


def text_run(text: str) -> str:
    """生成 Word 文本 run。"""
    return f'<w:r><w:t xml:space="preserve">{escape(text)}</w:t></w:r>'


def paragraph(text: str = "", style: str | None = None) -> str:
    """生成 Word 段落。"""
    style_xml = f'<w:pPr><w:pStyle w:val="{style}"/></w:pPr>' if style else ""
    return f"<w:p>{style_xml}{text_run(text)}</w:p>"


def bullet(text: str) -> str:
    """用文本项目符号表示项目行。"""
    return paragraph(f"• {text}")


def code_block(code: str) -> str:
    """把多行命令写成等宽风格段落。"""
    return "".join(paragraph(line, "CodeBlock") for line in code.strip().splitlines())


def build_section() -> str:
    """生成要追加到总文档末尾的新章节。"""
    parts: list[str] = []
    parts.append(paragraph(SECTION_MARKER, "Heading1"))
    parts.append(paragraph("本节记录 2026-06-03 增加的 DPVO 短期增强功能。这个功能的目标，是在不修改 DPVO 官方 CUDA/网络核心的前提下，让当前项目先具备类似 KV-Tracker 思想中的“关键帧记忆、跟踪质量评估和重定位候选”能力。它不是完整复刻 KV-Tracker，因为 KV-Tracker 官方核心代码目前没有公开；本节实现的是适合当前项目落地的第一版工程增强。"))

    parts.append(paragraph("14.1 为什么这样做", "Heading2"))
    parts.append(paragraph("DPVO 本身已经能输出单目相机 6DoF 轨迹，但原始 demo 更偏离线演示：跑完后得到轨迹和点云，缺少对每段跟踪质量、关键帧记忆和失败片段的结构化记录。商场第一人称视频里有人群、地面反光、运动模糊和重复纹理，即使使用遮挡后视频，也需要知道哪些片段可靠、哪些片段需要后续重定位或局部优化。"))
    parts.append(paragraph("因此，本次增强把 input_video.mp4_bev.mp4 作为默认输入，并在 DPVO 结束后自动生成一份增强结果目录。这个目录把轨迹、关键帧、质量曲线、低质量事件和重定位候选都保存下来，方便后续继续接 PnP、局部 BA 或更接近 KV-Tracker 的场景记忆模块。"))

    parts.append(paragraph("14.2 新增和修改的代码", "Heading2"))
    parts.append(bullet("新增：src/dpvo_localization/dpvo_localization/dpvo_enhancement.py。它负责读取 DPVO TUM 轨迹和遮挡视频，生成关键帧记忆、质量评估、低质量事件和重定位候选。"))
    parts.append(bullet("修改：src/dpvo_localization/dpvo_localization/run_dpvo_video.py。它现在默认使用 resources/input_video.mp4_bev.mp4，并在 DPVO 结束后自动调用增强模块。"))
    parts.append(bullet("修改：scripts/build_ros2_clean.sh。构建时增加 --base-paths src，避免 colcon 扫描 thirdparty/gsplat 和本地 Python 依赖目录。"))
    parts.append(bullet("保持不变：Opensource code/DPVO-main/demo.py、DPVO CUDA 扩展和 dpvo.pth 权重没有被改动。"))

    parts.append(paragraph("14.3 增强模块做了什么", "Heading2"))
    parts.append(bullet("读取 DPVO 保存的 saved_trajectories/mall_dpvo.txt，解析每一帧的时间戳、位置和四元数。"))
    parts.append(bullet("根据 stride 和 skip 估算每个 DPVO 样本对应的原始视频帧号。"))
    parts.append(bullet("读取 resources/input_video.mp4_bev.mp4，并按 DPVO stream.py 的逻辑缩放到 0.5、裁剪到 16 的倍数、识别黄/绿遮挡区域并填成中性灰。"))
    parts.append(bullet("计算每个样本的平移步长、旋转步长、Laplacian 模糊度、遮挡比例、重复位姿计数和综合质量分数。"))
    parts.append(bullet("按位姿变化、质量分数和最大间隔挑选关键帧，保存关键帧图片和 ORB 描述子。"))
    parts.append(bullet("把 weak/lost 连续片段整理成 tracking_events.json。"))
    parts.append(bullet("对低质量片段用 ORB 描述子匹配关键帧，输出 relocalization_candidates.json，作为后续 PnP 重定位的候选入口。"))

    parts.append(paragraph("14.4 当前质量分数的含义", "Heading2"))
    parts.append(paragraph("当前版本没有直接读取 DPVO 内部的 patch 数量或 BA 残差，所以质量分数是工程层面的估计分数，不是 DPVO 官方置信度。它主要由四部分组成：图像清晰度、运动跳变、遮挡比例和重复位姿计数。"))
    parts.append(code_block("""
quality_score =
  0.35 * blur_score
+ 0.25 * motion_score
+ 0.20 * mask_score
+ 0.20 * stale_score

quality_score >= 0.65 记为 good
0.35 <= quality_score < 0.65 记为 weak
quality_score < 0.35 记为 lost
"""))
    parts.append(paragraph("其中 blur_score 来自 Laplacian 方差，motion_score 用于惩罚异常平移或旋转跳变，mask_score 用于惩罚遮挡区域比例过高，stale_score 用于惩罚连续多帧位姿几乎不变的情况。"))

    parts.append(paragraph("14.5 当前测试结果", "Heading2"))
    parts.append(bullet("输入视频：resources/input_video.mp4_bev.mp4。"))
    parts.append(bullet("轨迹文件：Opensource code/DPVO-main/saved_trajectories/mall_dpvo.txt。"))
    parts.append(bullet("增强输出目录：output/dpvo_enhanced/mall_dpvo。"))
    parts.append(bullet("位姿样本数：1590。"))
    parts.append(bullet("关键帧数量：126。"))
    parts.append(bullet("质量事件数量：154。"))
    parts.append(bullet("good/weak/lost：good=1032，weak=558，lost=0。"))
    parts.append(bullet("平均质量分数：0.736。"))
    parts.append(bullet("DPVO 尺度下路径长度：4.629268。"))
    parts.append(paragraph("从结果看，遮挡后视频没有出现 lost 片段，但仍有 558 个样本被标为 weak，说明遮挡行人和地面反光确实有帮助，但商场视频中的模糊、转弯和重复纹理仍然会影响稳定性。"))

    parts.append(paragraph("14.6 输出文件说明", "Heading2"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/summary.json：增强分析总摘要，记录输入、输出、关键帧数量和质量统计。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/summary.txt：适合直接阅读的短摘要。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/tracking_quality.csv：每个 DPVO 样本的质量指标表格。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/tracking_quality.json：每个 DPVO 样本的质量指标 JSON。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/tracking_quality.png：质量分数、遮挡比例和模糊度曲线。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/keyframes/keyframes.json：关键帧列表。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/keyframes/images/：关键帧图像。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/keyframes/features/：关键帧 ORB 特征和描述子。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/tracking_events.json：weak/lost 片段列表。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/relocalization_candidates.json：每个低质量片段对应的候选关键帧。"))

    parts.append(paragraph("14.7 运行方法", "Heading2"))
    parts.append(paragraph("正常运行 DPVO，并在结束后自动生成增强结果："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 launch /home/ros/ros2_orbslam3/launch/dpvo_offline.launch.py
"""))
    parts.append(paragraph("如果已经有 saved_trajectories/mall_dpvo.txt，不想重新跑 DPVO，只想重新生成增强分析："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 run dpvo_localization run_dpvo_video \\
  --only_enhance \\
  --imagedir resources/input_video.mp4_bev.mp4 \\
  --name mall_dpvo \\
  --stride 2
"""))
    parts.append(paragraph("如果只想跑原始 DPVO，不想生成增强结果："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 run dpvo_localization run_dpvo_video --no_enhance
"""))

    parts.append(paragraph("14.8 和 KV-Tracker 思想的关系", "Heading2"))
    parts.append(paragraph("KV-Tracker 的核心是把多视图 Transformer 的 Key/Value cache 当作场景记忆，新帧通过 Query 去查询这份记忆，从而实现快速 6DoF tracking。DPVO 不是 Transformer 多视图模型，所以当前项目不能直接拿到同样的 K/V cache。"))
    parts.append(paragraph("本次增强借鉴的是它的工程思想：保留关键帧、建立场景记忆、评估新帧质量、为低质量片段提供回到历史关键帧的候选入口。当前记忆内容是显式的关键帧图像、DPVO 位姿和 ORB 描述子，而不是 Transformer 隐空间 K/V。"))
    parts.append(paragraph("后续如果要更接近 KV-Tracker，可以继续做三步：第一，把关键帧 ORB 匹配升级成 2D-3D PnP 重定位；第二，把关键帧和 DPVO 局部点云绑定，形成可优化 local map；第三，等待 KV-Tracker 或 Pi3/Pi3X 相关代码成熟后，加入真正的学习型场景记忆。"))

    parts.append(paragraph("14.9 当前限制和下一步", "Heading2"))
    parts.append(bullet("当前增强模块是后处理，不会在 DPVO 运行中主动纠正轨迹。"))
    parts.append(bullet("当前重定位候选只输出候选关键帧，还没有执行 PnP 和局部 BA。"))
    parts.append(bullet("质量分数是工程估计值，不是 DPVO 官方网络内部置信度。"))
    parts.append(bullet("下一步建议实现 relocalizer.py：读取 keyframes/features，建立 2D-3D 对应关系，使用 solvePnPRansac 给 weak/lost 片段估计候选位姿。"))
    parts.append(bullet("再下一步建议把 DPVO 内部点云或保存的 PLY 与关键帧特征绑定，实现真正可用于恢复的局部地图。"))

    return "".join(parts)


def build_second_section() -> str:
    """生成 DPVO 第二版增强章节。"""
    parts: list[str] = []
    parts.append(paragraph(SECOND_SECTION_MARKER, "Heading1"))
    parts.append(paragraph("本节记录 2026-06-03 继续完成的 DPVO 第二版增强。第一版只给 weak/lost 片段提供候选关键帧；第二版进一步对这些候选做 ORB 匹配、Essential Matrix、RANSAC 和 recoverPose 几何验证。它的目的不是直接替代 DPVO，也不是伪造 PnP，而是判断：某个弱跟踪片段是否还能和历史关键帧形成稳定的几何约束。"))

    parts.append(paragraph("15.1 为什么不是直接 PnP", "Heading2"))
    parts.append(paragraph("PnP 的完整输入是 2D-3D 对应关系，也就是当前图像上的二维点 u_i，需要知道它对应地图里的三维点 X_i，然后通过 solvePnPRansac 求相机位姿。当前项目已有关键帧图像、关键帧 ORB 特征和 DPVO 轨迹，但还没有把每个 ORB 特征绑定到 DPVO 的三维地图点。"))
    parts.append(paragraph("如果在没有真实 3D 点关联的情况下硬做 PnP，只能使用伪深度或假三维点，这会让结果看起来像 PnP，实际没有可靠几何意义。因此第二版先做 2D-2D 几何验证：用两张图中的匹配点估计 Essential Matrix，判断它们是否满足同一个相机运动模型。"))

    parts.append(paragraph("15.2 第二版新增输出", "Heading2"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/relocalization_results.json：每个 weak/lost 事件的所有候选关键帧几何验证结果。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/relocalization_results.csv：适合表格查看的几何验证摘要。"))
    parts.append(bullet("output/dpvo_enhanced/mall_dpvo/relocalization_report.txt：适合直接阅读的几何验证报告。"))

    parts.append(paragraph("15.3 第二版当前测试结果", "Heading2"))
    parts.append(bullet("输入视频：resources/input_video.mp4_bev.mp4。"))
    parts.append(bullet("位姿样本数：1590。"))
    parts.append(bullet("关键帧数量：154。"))
    parts.append(bullet("质量事件数量：175。"))
    parts.append(bullet("good/weak/lost：good=1018，weak=572，lost=0。"))
    parts.append(bullet("平均质量分数：0.741。"))
    parts.append(bullet("几何验证通过事件数：165。"))
    parts.append(bullet("几何验证通过比例：0.943。"))
    parts.append(bullet("最佳候选状态统计：verified=165，pose_degenerate_or_low_parallax=8，geometry_weak=2。"))
    parts.append(paragraph("这说明大多数 weak 片段并不是完全不可恢复，而是仍然能和历史关键帧形成稳定的 2D-2D 几何关系。剩下的 8 个低视差/退化事件，通常表示画面和关键帧太相似或相对运动太小，recoverPose 无法恢复可靠平移；剩下 2 个 geometry_weak 事件才更像真正匹配较弱的片段。"))

    parts.append(paragraph("15.4 几何验证原理", "Heading2"))
    parts.append(paragraph("第二版使用的是经典两视图几何流程。先用 ORB 描述子做 KNN 匹配，再用 Lowe ratio test 去掉明显错误匹配；然后使用 RANSAC 估计 Essential Matrix，最后用 recoverPose 从 Essential Matrix 中恢复相对旋转和平移方向。"))
    parts.append(code_block("""
ORB 匹配：
当前弱帧 keypoints/descriptors <-> 候选关键帧 keypoints/descriptors

Essential Matrix 约束：
x_2^T E x_1 = 0

RANSAC：
反复抽样估计 E，保留满足几何约束的内点

recoverPose：
E -> R, t_direction
"""))
    parts.append(paragraph("这里的 R 和 t_direction 是两帧之间的相对运动，其中 t 只有方向，没有真实尺度。单目相机没有深度，所以两视图几何不能直接给真实米制位移。"))

    parts.append(paragraph("15.5 verified / pose_degenerate / geometry_weak 是什么", "Heading2"))
    parts.append(bullet("verified：ORB 匹配数量、Essential Matrix 内点数量、内点比例和 recoverPose 内点数量都满足阈值，说明该 weak 片段可以和某个关键帧形成稳定几何约束。"))
    parts.append(bullet("pose_degenerate_or_low_parallax：匹配和 Essential Matrix 看起来很好，但 recoverPose 无法恢复足够可靠的相对运动，常见原因是两帧太接近、视差太小或运动退化。"))
    parts.append(bullet("geometry_weak：匹配点或 RANSAC 内点不足，说明这个 weak 片段和候选关键帧之间几何关系不够可靠。"))

    parts.append(paragraph("15.6 运行方法", "Heading2"))
    parts.append(paragraph("正常运行 DPVO 时会自动执行第二版增强："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 launch /home/ros/ros2_orbslam3/launch/dpvo_offline.launch.py
"""))
    parts.append(paragraph("如果只想用已有轨迹重新跑增强和几何验证："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 run dpvo_localization run_dpvo_video \\
  --only_enhance \\
  --imagedir resources/input_video.mp4_bev.mp4 \\
  --name mall_dpvo \\
  --stride 2
"""))
    parts.append(paragraph("如果想加速几何验证，可以减少每个事件检查的候选关键帧数量："))
    parts.append(code_block("""
scripts/run_ros2_clean.sh ros2 run dpvo_localization run_dpvo_video \\
  --only_enhance \\
  --imagedir resources/input_video.mp4_bev.mp4 \\
  --name mall_dpvo \\
  --stride 2 \\
  --relocalization-topk 3
"""))

    parts.append(paragraph("15.7 面向初学者的代码理解", "Heading2"))
    parts.append(paragraph("当前 DPVO 增强主要在 dpvo_enhancement.py 中完成。可以按模块理解，而不是一上来逐行死记。parse_args 负责读取命令行参数；PoseSample 和 CameraCalib 是 dataclass，用来保存一帧轨迹和相机标定；read_tum_trajectory 读取 DPVO 轨迹；prepare_frame_like_dpvo 模拟 DPVO 的视频预处理；score_quality 计算质量分数；select_keyframes 挑选关键帧；compute_relocalization_candidates 用 ORB 匹配找候选；compute_geometric_relocalization_results 做第二版几何验证；write_summary 写出最终报告。"))
    parts.append(paragraph("如果你不熟悉 Python，可以先掌握四个概念：Path 表示文件路径；list/dict 分别表示列表和字典；numpy array 表示矩阵和向量；函数 def xxx(...) 是把一段操作封装起来反复调用。C++ 部分暂时不用动，因为当前增强都在 Python 包装层完成，没有修改 ORB-SLAM3 的 C++ 节点。"))

    parts.append(paragraph("15.8 下一步", "Heading2"))
    parts.append(bullet("把 DPVO 输出的三维点和关键帧 ORB 特征建立关联，形成真正的 2D-3D 地图。"))
    parts.append(bullet("在 verified 的 weak 片段上执行 solvePnPRansac，输出候选相机位姿。"))
    parts.append(bullet("用局部 BA 优化关键帧位姿和地图点，把候选位姿变成可用于修正轨迹的结果。"))
    parts.append(bullet("把 quality_score、verified 状态和关键帧候选做成实时 viewer 或 ROS2 topic。"))
    return "".join(parts)


def build_third_section() -> str:
    """生成 DPVO 第三版可视化增强章节。"""
    parts: list[str] = []
    parts.append(paragraph(THIRD_SECTION_MARKER, "Heading1"))
    parts.append(paragraph("本节记录 2026-06-03 继续完成的 DPVO 第三版增强。第三版的目标，是让第二版的质量评估、关键帧记忆和几何重定位验证不再只停留在 JSON/CSV/TXT 表格里，而是能通过实时窗口和 MP4 结果视频直接看到。这样后续调参时，可以一边看第一人称画面，一边看当前帧质量、weak/lost 事件、几何验证结果和轨迹小地图。"))

    parts.append(paragraph("16.1 第三版新增内容", "Heading2"))
    parts.append(bullet("修改：src/dpvo_localization/dpvo_localization/dpvo_enhancement.py。新增增强可视化渲染、MP4 视频写出、截图保存和实时窗口显示。"))
    parts.append(bullet("修改：src/dpvo_localization/dpvo_localization/run_dpvo_video.py。新增 --enhanced-show、--no-enhanced-video、--enhanced-visualization-every 等 ROS2 入口参数。"))
    parts.append(bullet("新增输出：output/dpvo_enhanced/mall_dpvo/visualization/dpvo_enhanced_visualization.mp4。"))
    parts.append(bullet("新增输出：output/dpvo_enhanced/mall_dpvo/visualization/snapshots/，保存若干张代表性可视化截图。"))
    parts.append(bullet("新增输出：output/dpvo_enhanced/mall_dpvo/visualization/visualization_summary.json，记录视频帧数、帧率和输出路径。"))

    parts.append(paragraph("16.2 当前完整测试结果", "Heading2"))
    parts.append(bullet("输入视频：/home/ros/ros2_orbslam3/resources/input_video.mp4_bev.mp4。"))
    parts.append(bullet("输出目录：/home/ros/ros2_orbslam3/output/dpvo_enhanced/mall_dpvo。"))
    parts.append(bullet("增强可视化视频：/home/ros/ros2_orbslam3/output/dpvo_enhanced/mall_dpvo/visualization/dpvo_enhanced_visualization.mp4。"))
    parts.append(bullet("增强可视化截图目录：/home/ros/ros2_orbslam3/output/dpvo_enhanced/mall_dpvo/visualization/snapshots。"))
    parts.append(bullet("视频帧数：1590。"))
    parts.append(bullet("视频大小：约 75MB。"))
    parts.append(bullet("视频尺寸：1420 x 700。"))
    parts.append(bullet("视频帧率：约 14.709 FPS，由输入视频 FPS / stride 自动估计。"))
    parts.append(bullet("几何验证通过事件数：165 / 175，通过比例 0.943。"))
    parts.append(paragraph("这次生成的视频已经用 OpenCV 读回验证：VideoCapture 可以正常打开，第一帧可以正常读取。"))

    parts.append(paragraph("16.3 可视化画面怎么看", "Heading2"))
    parts.append(bullet("左侧是当前输入视频帧，已经按 DPVO 预处理逻辑缩放、去畸变、裁剪到 16 的倍数，并把遮挡区域用绿色半透明区域标出。"))
    parts.append(bullet("左上角显示 sample、原始视频 frame、当前状态 good/weak/lost、quality、blur 和 mask ratio。"))
    parts.append(bullet("边框颜色表示跟踪状态：绿色是 good，黄色是 weak，红色是 lost。"))
    parts.append(bullet("右侧面板显示当前时间、质量分数、单步平移、单步旋转、遮挡比例和模糊度。"))
    parts.append(bullet("右侧 quality 条里有 0.35 和 0.65 两条阈值线；低于 0.35 是 lost，0.35 到 0.65 是 weak，高于 0.65 是 good。"))
    parts.append(bullet("右侧 trajectory x-z 小地图是 DPVO 轨迹的俯视图；浅色线表示全局轨迹，亮色线表示当前已经走过的轨迹，黄色圆点表示当前帧位置。"))
    parts.append(bullet("如果当前帧处于 weak/lost 事件中，右侧会显示 event id、verified、status、best keyframe、matches、inliers 和 inlier ratio。"))

    parts.append(paragraph("16.4 运行方法", "Heading2"))
    parts.append(paragraph("完整运行已有轨迹的增强分析，并生成完整 MP4 可视化视频："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 run dpvo_localization run_dpvo_video \\
  --only_enhance \\
  --imagedir resources/input_video.mp4_bev.mp4 \\
  --name mall_dpvo \\
  --stride 2
"""))
    parts.append(paragraph("如果想实时看增强可视化窗口，同时保存 MP4："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 run dpvo_localization run_dpvo_video \\
  --only_enhance \\
  --imagedir resources/input_video.mp4_bev.mp4 \\
  --name mall_dpvo \\
  --stride 2 \\
  --enhanced-show
"""))
    parts.append(paragraph("如果只想快速预览，减少几何候选数量并抽帧写视频："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 run dpvo_localization run_dpvo_video \\
  --only_enhance \\
  --imagedir resources/input_video.mp4_bev.mp4 \\
  --name mall_dpvo \\
  --stride 2 \\
  --relocalization-topk 1 \\
  --enhanced-visualization-every 25 \\
  --enhanced-output-dir output/dpvo_enhanced_visual_test
"""))
    parts.append(paragraph("如果只想重新生成表格和 JSON，不想保存视频："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 run dpvo_localization run_dpvo_video \\
  --only_enhance \\
  --imagedir resources/input_video.mp4_bev.mp4 \\
  --name mall_dpvo \\
  --stride 2 \\
  --no-enhanced-video
"""))

    parts.append(paragraph("16.5 和 KV-Track3r 论文的关系", "Heading2"))
    parts.append(paragraph("你提供的论文 KV_Track3r.pdf 标题为 KV-Tracker: Real-Time Pose Tracking with Transformers。论文的核心思想是：先用一组选出的关键帧进行 mapping，生成多视图 Transformer 全局注意力层里的 Key/Value 缓存；随后 tracking 时，新来的单帧作为 query，只和已经缓存的 KV 场景表示做注意力交互，从而避免每来一帧就重新对所有关键帧做完整全局注意力计算。论文中还提到该方法可以达到接近实时的 6DoF 跟踪，并且在物体模式下可以结合分割 mask，把背景遮掉。"))
    parts.append(paragraph("当前项目使用的是 DPVO，不是 π3/VGGT 这类多视图 Transformer，所以我们不能直接得到论文里的神经网络 KV-cache。第三版增强借鉴的是它的系统思想，而不是声称复刻其网络：用关键帧作为场景记忆，用质量分数判断是否可靠，用候选关键帧和几何验证帮助 weak 片段回到历史记忆，并通过实时可视化让这个过程可观察。"))
    parts.append(bullet("KV-Track3r 的 keyframes/KV-cache，在当前项目里对应 keyframes/images、keyframes/features 和 DPVO 位姿。"))
    parts.append(bullet("KV-Track3r 的 query frame tracking，在当前项目里对应当前 weak 帧提取 ORB 后去查历史关键帧。"))
    parts.append(bullet("KV-Track3r 的 mask object/background，在当前项目里对应 input_video.mp4_bev.mp4 中对行人和地面反光区域的遮挡，并在增强模块里继续检测这些遮挡区域。"))
    parts.append(bullet("KV-Track3r 的 low confidence keyframe rejection，在当前项目里对应 keyframe_min_quality：质量太低的帧不会进入关键帧记忆。"))

    parts.append(paragraph("16.6 主要代码解释", "Heading2"))
    parts.append(bullet("write_visualization_outputs：可视化输出主函数。它顺序读取输入视频，找到每个 DPVO 样本对应的原始视频帧，调用渲染函数生成画布，然后写入 MP4、保存截图，必要时打开实时窗口。"))
    parts.append(bullet("render_visualization_canvas：把一帧视频、当前质量分数、事件信息、重定位结果和轨迹小地图合成到一张图里。"))
    parts.append(bullet("draw_trajectory_map：把 DPVO 的三维位置投影到 x-z 平面，画成右侧小地图。"))
    parts.append(bullet("draw_quality_bar：画当前帧质量条，并标出 0.35 和 0.65 两条阈值。"))
    parts.append(bullet("build_event_lookup：把 tracking_events.json 里的连续 weak/lost 片段展开成 sample_index 到 event 的查询表，方便实时渲染时判断当前帧属于哪个事件。"))
    parts.append(bullet("select_snapshot_samples：自动挑选少量代表性 sample，保存为 JPG 截图，方便不用打开 MP4 也能快速查看效果。"))
    parts.append(bullet("run_dpvo_video.py 中新增的 --enhanced-show、--no-enhanced-video 和 --enhanced-visualization-every 参数，只是把用户命令转发给 dpvo_enhancement.py。真正画图和写视频的逻辑在 dpvo_enhancement.py 里。"))

    parts.append(paragraph("16.7 当前限制和下一步", "Heading2"))
    parts.append(bullet("当前实时窗口是增强后处理回放，不是侵入 DPVO 网络内部的在线可视化。也就是说，DPVO 仍然先输出轨迹，增强模块再读取轨迹和视频生成可视化。"))
    parts.append(bullet("当前还没有把 verified 结果反向写回 DPVO 轨迹，所以它能帮助你判断哪里可靠、哪里弱，但还不会主动修正轨迹。"))
    parts.append(bullet("下一步真正接近 KV-Track3r/KV-Tracker 效果的路线，是把关键帧特征绑定到 DPVO 三维点，然后对 weak 帧执行 solvePnPRansac，再用局部 BA 修正位姿。"))
    parts.append(bullet("如果后续拿到 KV-Track3r 官方代码或 π3/VGGT 可用跟踪接口，可以把当前关键帧/质量/可视化框架保留，把显式 ORB 记忆替换或融合为神经网络 KV-cache 记忆。"))
    return "".join(parts)


def build_fourth_section() -> str:
    """生成 DPVO 三维增强可视化章节。"""
    parts: list[str] = []
    parts.append(paragraph(FOURTH_SECTION_MARKER, "Heading1"))
    parts.append(paragraph("本节记录在二维增强视频之外新增的三维增强可视化。这个功能的目标，是让结果更接近论文和作者 demo 中常见的三维跟踪展示风格：在 3D 坐标系里显示相机运动轨迹、关键帧、当前相机视锥、weak/verified 状态和可选点云。原来的二维视频仍然保留，不会被覆盖。"))

    parts.append(paragraph("17.1 当前已经生成的 3D 结果", "Heading2"))
    parts.append(bullet("脚本：scripts/render_dpvo_3d_visualization.py。"))
    parts.append(bullet("输出目录：/home/ros/ros2_orbslam3/output/dpvo_enhanced/mall_dpvo/visualization_3d。"))
    parts.append(bullet("3D 视频：/home/ros/ros2_orbslam3/output/dpvo_enhanced/mall_dpvo/visualization_3d/dpvo_enhanced_3d.mp4。"))
    parts.append(bullet("3D 截图目录：/home/ros/ros2_orbslam3/output/dpvo_enhanced/mall_dpvo/visualization_3d/snapshots。"))
    parts.append(bullet("3D 摘要：/home/ros/ros2_orbslam3/output/dpvo_enhanced/mall_dpvo/visualization_3d/summary.txt。"))
    parts.append(bullet("当前写入帧数：399。"))
    parts.append(bullet("输出帧率：12 FPS。"))
    parts.append(bullet("输出尺寸：1280 x 720。"))
    parts.append(bullet("当前点云：未加载；当前显示的是三维轨迹、关键帧和相机视锥。"))

    parts.append(paragraph("17.2 3D 画面怎么看", "Heading2"))
    parts.append(bullet("青色轨迹表示已经走过的 DPVO 相机运动轨迹。"))
    parts.append(bullet("灰色轨迹表示完整轨迹背景。"))
    parts.append(bullet("绿色点或绿色相机视锥表示当前帧状态为 good。"))
    parts.append(bullet("黄色表示 weak，红色表示 lost。"))
    parts.append(bullet("紫色小相机视锥表示历史关键帧相机。"))
    parts.append(bullet("如果当前帧属于 weak/lost 事件，右上角会显示 event、verified、status、inliers 和 inlier ratio。"))
    parts.append(bullet("如果加载了 PLY 点云，画面中会额外出现稀疏或半稠密场景点云。"))

    parts.append(paragraph("17.3 当前效果和作者 live_demo 的区别", "Heading2"))
    parts.append(paragraph("作者视频 live_demo 通常会同时显示相机轨迹、关键帧和重建出的场景几何，例如点云、局部 point map 或神经渲染结果。当前 DPVO 增强主结果里只有轨迹、关键帧和质量分析，还没有保存 DPVO 点云 PLY，所以目前 3D 视频是“3D 跟踪回放”，不是完整“3D 场景建模”。"))
    parts.append(paragraph("这不是代码错误，而是输入数据层级不同：轨迹只能画相机怎么走，点云/深度/3DGS 才能画商场墙面、地面和物体。后续只要获得 DPVO PLY 或融合 VGGT 点云，就可以把点云作为背景叠加到同一个 3D viewer 中。"))

    parts.append(paragraph("17.4 运行方法", "Heading2"))
    parts.append(paragraph("生成当前这种三维轨迹/相机回放："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
/home/ros/miniconda3/envs/dpvo/bin/python scripts/render_dpvo_3d_visualization.py \\
  --enhanced-dir output/dpvo_enhanced/mall_dpvo \\
  --trajectory "Opensource code/DPVO-main/saved_trajectories/mall_dpvo.txt" \\
  --name mall_dpvo \\
  --every 4 \\
  --fps 12 \\
  --width 1280 \\
  --height 720
"""))
    parts.append(paragraph("如果 WSL 图形界面可用，希望渲染时实时弹窗预览："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
/home/ros/miniconda3/envs/dpvo/bin/python scripts/render_dpvo_3d_visualization.py \\
  --enhanced-dir output/dpvo_enhanced/mall_dpvo \\
  --trajectory "Opensource code/DPVO-main/saved_trajectories/mall_dpvo.txt" \\
  --name mall_dpvo \\
  --every 4 \\
  --show
"""))
    parts.append(paragraph("如果后续重新跑 DPVO 并保存了 PLY 点云，可以这样加载点云："))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
/home/ros/miniconda3/envs/dpvo/bin/python scripts/render_dpvo_3d_visualization.py \\
  --enhanced-dir output/dpvo_enhanced/mall_dpvo \\
  --trajectory "Opensource code/DPVO-main/saved_trajectories/mall_dpvo.txt" \\
  --name mall_dpvo \\
  --pointcloud "Opensource code/DPVO-main/mall_dpvo.ply"
"""))

    parts.append(paragraph("17.5 如何得到 DPVO PLY 点云", "Heading2"))
    parts.append(paragraph("DPVO 官方 demo.py 支持 --save_ply。当前 ROS2 包装脚本已经保留了 --save_ply 参数。如果你想让 3D viewer 里出现 DPVO 自己的点云，需要重新跑一次 DPVO，并加上 --save_ply。"))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 run dpvo_localization run_dpvo_video \\
  --imagedir resources/input_video.mp4_bev.mp4 \\
  --name mall_dpvo \\
  --stride 2 \\
  --save_ply
"""))
    parts.append(paragraph("DPVO demo.py 会在 DPVO 源码运行目录下保存 mall_dpvo.ply。正常路径应为：/home/ros/ros2_orbslam3/Opensource code/DPVO-main/mall_dpvo.ply。保存完成后，再运行上一节带 --pointcloud 的 3D 渲染命令。"))

    parts.append(paragraph("17.6 主要代码解释", "Heading2"))
    parts.append(bullet("read_tum_trajectory：读取 DPVO 的 TUM 轨迹，获得每一帧相机位置和四元数。"))
    parts.append(bullet("merge_quality：读取 tracking_quality.csv，把 quality_score、good/weak/lost、遮挡比例和模糊度合并到轨迹样本中。"))
    parts.append(bullet("read_ply_points：读取可选 PLY 点云。如果没有点云，脚本仍然可以生成 3D 轨迹回放。"))
    parts.append(bullet("draw_camera_frustum：根据当前相机位置和四元数画一个相机视锥，这就是 3D 视频里像小摄像机一样的东西。"))
    parts.append(bullet("render_frame：渲染每一帧 3D 画面，包括轨迹、关键帧、当前相机、事件状态和可选点云。"))
    parts.append(bullet("main：负责组织输入文件、创建输出目录、循环渲染每一帧、写 MP4 和截图。"))

    parts.append(paragraph("17.7 后续升级路线", "Heading2"))
    parts.append(bullet("第一步：重新运行 DPVO 并使用 --save_ply，得到 DPVO 稀疏点云。"))
    parts.append(bullet("第二步：把 DPVO PLY 加载到 3D viewer 中，形成轨迹 + 相机 + 点云的三维结果。"))
    parts.append(bullet("第三步：如果希望墙壁和地面更完整，需要使用 VGGT 点云、COLMAP/BA 或 3DGS，而不是只依赖 DPVO 轨迹。"))
    parts.append(bullet("第四步：如果后续拿到 KV-Track3r 官方实现，可以把当前的 3D viewer 作为外层展示框架，把内部数据源替换为 KV-cache tracking 输出。"))
    return "".join(parts)


def build_fifth_section() -> str:
    """生成三维坐标对齐和作者 demo 风格升级路线章节。"""
    parts: list[str] = []
    parts.append(paragraph(FIFTH_SECTION_MARKER, "Heading1"))
    parts.append(paragraph("本节解释为什么第一版 3D 增强地图看起来像一直向上走，以及后处理上应该加入什么才能更接近作者 live_demo 的展示效果。核心结论是：DPVO/单目 VO 的输出坐标不是现实世界重力坐标，不能直接把原始 z 轴当成竖直方向。"))

    parts.append(paragraph("18.1 为什么旧 3D 地图看起来违背物理常识", "Heading2"))
    parts.append(paragraph("旧版 3D viewer 直接使用 DPVO 的原始 x/y/z，并把 z 轴画成三维图里的竖直方向。但单目视觉里程计的世界坐标系是算法内部坐标系，它没有天然重力方向，也没有天然地面平面约束。"))
    parts.append(paragraph("对 mall_dpvo 轨迹做统计后可以看到：x 范围约 0.95，y 范围约 1.46，z 范围约 5.95。z 轴变化最大，更像沿商场前进的方向，而不是现实中的高度方向。因此把 z 画成竖直轴后，就会视觉上像一直向上走。"))
    parts.append(paragraph("这不是你实际运动方式变了，也不是 DPVO 一定错了，而是显示坐标没有做世界对齐。"))

    parts.append(paragraph("18.2 已新增的 trajectory_pca 对齐", "Heading2"))
    parts.append(paragraph("scripts/render_dpvo_3d_visualization.py 已经新增 --align-world 参数，默认使用 trajectory_pca。它会用轨迹 PCA 自动估计主要运动平面，把最大运动方向放到水平 x 轴，把第二大运动方向放到水平 y 轴，把最小变化方向作为显示中的竖直 z 轴。"))
    parts.append(paragraph("新的对齐版输出目录：/home/ros/ros2_orbslam3/output/dpvo_enhanced/mall_dpvo/visualization_3d_aligned。"))
    parts.append(paragraph("新的对齐版视频：/home/ros/ros2_orbslam3/output/dpvo_enhanced/mall_dpvo/visualization_3d_aligned/dpvo_enhanced_3d.mp4。"))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
/home/ros/miniconda3/envs/dpvo/bin/python scripts/render_dpvo_3d_visualization.py \\
  --enhanced-dir output/dpvo_enhanced/mall_dpvo \\
  --trajectory "Opensource code/DPVO-main/saved_trajectories/mall_dpvo.txt" \\
  --name mall_dpvo \\
  --every 4 \\
  --fps 12 \\
  --width 1280 \\
  --height 720 \\
  --output-dir output/dpvo_enhanced/mall_dpvo/visualization_3d_aligned \\
  --align-world trajectory_pca
"""))
    parts.append(paragraph("如果想查看未经校正的 DPVO 原始坐标，可以使用 --align-world raw，但一般不建议用 raw 来判断物理运动，因为它容易把前进方向误解成竖直方向。"))

    parts.append(paragraph("18.3 想做成作者 live_demo，后处理应加什么", "Heading2"))
    parts.append(bullet("第一步：世界坐标对齐。当前已经完成 trajectory_pca，它解决前进方向被画成竖直方向的问题。"))
    parts.append(bullet("第二步：重力/地面约束。如果后续有 IMU、地面分割或楼梯区域标签，应加入 gravity alignment、floor plane fitting 和 stair segment height prior。这样才能更准确地区分平地行走、下楼和转弯。"))
    parts.append(bullet("第三步：轨迹平滑和尺度校正。单目 DPVO 没有真实米制尺度，后处理可以加入 Sim(3) 尺度归一化、pose smoothing 和 flat-floor constant-height prior。"))
    parts.append(bullet("第四步：点云或场景几何。作者 demo 的高级感来自轨迹之外的场景几何。下一步应加载 DPVO --save_ply 点云、VGGT 点云、COLMAP + BA 点云或 3DGS 渲染结果。"))
    parts.append(bullet("第五步：关键帧图像平面。为了更像论文视频，可以把关键帧图片贴到相机视锥前方，形成轨迹 + 相机 + 关键帧小图 + 点云背景的展示。"))

    parts.append(paragraph("18.4 当前最推荐的下一步", "Heading2"))
    parts.append(paragraph("最实用的下一步是重新跑 DPVO 并加 --save_ply，生成 DPVO 稀疏点云，然后把这个 PLY 加载到当前 3D viewer。这样会从“3D 轨迹回放”升级为“3D 轨迹 + 稀疏场景点云回放”。"))
    parts.append(code_block("""
cd /home/ros/ros2_orbslam3
scripts/run_ros2_clean.sh ros2 run dpvo_localization run_dpvo_video \\
  --imagedir resources/input_video.mp4_bev.mp4 \\
  --name mall_dpvo \\
  --stride 2 \\
  --save_ply

/home/ros/miniconda3/envs/dpvo/bin/python scripts/render_dpvo_3d_visualization.py \\
  --enhanced-dir output/dpvo_enhanced/mall_dpvo \\
  --trajectory "Opensource code/DPVO-main/saved_trajectories/mall_dpvo.txt" \\
  --name mall_dpvo \\
  --pointcloud "Opensource code/DPVO-main/mall_dpvo.ply" \\
  --align-world trajectory_pca
"""))
    return "".join(parts)


def update_document_xml(xml: str) -> str:
    """替换首页说明并追加新章节。"""
    xml = xml.replace(
        "日期：2026-05-30（本版补 VGGT 全局对齐后处理链路）",
        "日期：2026-06-03（本版补 DPVO 关键帧记忆与跟踪质量增强）",
    )
    xml = xml.replace(
        "日期：2026-06-03（本版补 DPVO 关键帧记忆与跟踪质量增强）",
        "日期：2026-06-03（本版补 DPVO 关键帧记忆、跟踪质量与几何重定位验证）",
    )
    xml = xml.replace(
        "日期：2026-06-03（本版补 DPVO 关键帧记忆、跟踪质量与几何重定位验证）",
        "日期：2026-06-03（本版补 DPVO 关键帧记忆、几何重定位验证与可视化视频）",
    )
    xml = xml.replace(
        "日期：2026-06-03（本版补 DPVO 关键帧记忆、几何重定位验证与可视化视频）",
        "日期：2026-06-03（本版补 DPVO 关键帧记忆、几何重定位验证、2D/3D 可视化视频）",
    )
    xml = xml.replace(
        "本版在最后追加了第 13 章「VGGT 多视图深度重建链路」，记录新的窗口对齐 + 全局点云渲染流程，便于和 DPVO/ORB-SLAM3 互补使用。",
        "本版在最后追加了第 14 章「DPVO 短期增强：关键帧记忆与跟踪质量评估」，记录基于 input_video.mp4_bev.mp4 的 DPVO 增强流程、输出结果和后续扩展路线。",
    )
    xml = xml.replace(
        "本版在最后追加了第 14 章「DPVO 短期增强：关键帧记忆与跟踪质量评估」，记录基于 input_video.mp4_bev.mp4 的 DPVO 增强流程、输出结果和后续扩展路线。",
        "本版在最后追加了第 14 章和第 15 章，记录 DPVO 关键帧记忆、跟踪质量评估、几何重定位验证、输出结果和后续 PnP/局部 BA 扩展路线。",
    )
    xml = xml.replace(
        "本版在最后追加了第 14 章和第 15 章，记录 DPVO 关键帧记忆、跟踪质量评估、几何重定位验证、输出结果和后续 PnP/局部 BA 扩展路线。",
        "本版在最后追加了第 14 章、第 15 章和第 16 章，记录 DPVO 关键帧记忆、跟踪质量评估、几何重定位验证、实时可视化、结果视频和后续 PnP/局部 BA 扩展路线。",
    )
    xml = xml.replace(
        "本版在最后追加了第 14 章、第 15 章和第 16 章，记录 DPVO 关键帧记忆、跟踪质量评估、几何重定位验证、实时可视化、结果视频和后续 PnP/局部 BA 扩展路线。",
        "本版在最后追加了第 14 章、第 15 章、第 16 章和第 17 章，记录 DPVO 关键帧记忆、跟踪质量评估、几何重定位验证、2D/3D 可视化、结果视频和后续 PnP/局部 BA 扩展路线。",
    )

    section = ""
    if SECTION_MARKER not in xml:
        section += build_section()
    if SECOND_SECTION_MARKER not in xml:
        section += build_second_section()
    if THIRD_SECTION_MARKER not in xml:
        section += build_third_section()
    if FOURTH_SECTION_MARKER not in xml:
        section += build_fourth_section()
    if FIFTH_SECTION_MARKER not in xml:
        section += build_fifth_section()
    if not section:
        return xml

    marker = "<w:sectPr"
    index = xml.rfind(marker)
    if index == -1:
        raise RuntimeError("word/document.xml 中找不到 w:sectPr，无法插入新章节")
    return xml[:index] + section + xml[index:]


def main() -> None:
    """脚本入口。"""
    if not DOCX.exists():
        raise FileNotFoundError(DOCX)

    if not BACKUP.exists():
        shutil.copy2(DOCX, BACKUP)

    with zipfile.ZipFile(DOCX, "r") as zin:
        entries = {name: zin.read(name) for name in zin.namelist()}

    document_xml = entries["word/document.xml"].decode("utf-8")
    entries["word/document.xml"] = update_document_xml(document_xml).encode("utf-8")

    if "docProps/core.xml" in entries:
        core_xml = entries["docProps/core.xml"].decode("utf-8")
        now = datetime.utcnow().isoformat() + "Z"
        import re

        core_xml = re.sub(
            r"<dcterms:modified[^>]*>.*?</dcterms:modified>",
            f'<dcterms:modified xsi:type="dcterms:W3CDTF">{now}</dcterms:modified>',
            core_xml,
        )
        entries["docProps/core.xml"] = core_xml.encode("utf-8")

    with tempfile.NamedTemporaryFile(delete=False, suffix=".docx") as tmp:
        temp_path = Path(tmp.name)

    with zipfile.ZipFile(temp_path, "w", compression=zipfile.ZIP_DEFLATED) as zout:
        for name, data in entries.items():
            zout.writestr(name, data)

    shutil.move(temp_path, DOCX)
    print(f"已更新: {DOCX}")
    print(f"备份文件: {BACKUP}")


if __name__ == "__main__":
    main()
