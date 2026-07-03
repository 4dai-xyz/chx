#!/usr/bin/env python3
"""
单目相机标定 (棋盘格法)

输入: 一段棋盘格视频 (工业 USB 相机录制的标定视频也可以)
输出:
  output/calib_report.txt        标定报告 (fx, fy, cx, cy, dist, RMS)
  output/calib_debug/*.jpg       前 12 张检测成功的可视化
  config/KannalaBrandt8.yaml     自动更新 (备份原文件到 .bak)

用法:
  python3 scripts/calibrate_camera.py --video /path/to/chessboard.mp4
  python3 scripts/calibrate_camera.py --board-w 8 --board-h 6 --square 0.025
  python3 scripts/calibrate_camera.py --frame-step 30 --max-views 40

参数:
  --video      棋盘格视频路径 (默认 resources/standardization.mp4)
  --board-w    棋盘内角点列数 (默认 8)
  --board-h    棋盘内角点行数 (默认 6)
  --square     单个方格物理边长 (米; 不影响内参, 只影响 RMS 像素单位)
  --frame-step 每隔 N 帧采样一次 (默认 30, 即每秒一帧)
  --max-views  最多收集多少个有效视图 (默认 40)
  --dry-run    只标定不写 yaml
"""
import argparse
import os
import sys
import shutil
import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VIDEO = os.path.join(REPO, 'resources', 'standardization.mp4')
YAML_PATH = os.path.join(REPO, 'config', 'KannalaBrandt8.yaml')
REPORT_PATH = os.path.join(REPO, 'output', 'calib_report.txt')
DEBUG_DIR = os.path.join(REPO, 'output', 'calib_debug')


def detect_corners(gray, board_size):
    """返回 (corners or None, refined or None)"""
    # 优先使用 OpenCV 新版 SB 棋盘检测，对模糊和倾斜更稳
    if hasattr(cv2, 'findChessboardCornersSBWithMeta'):
        try:
            ok, corners, _ = cv2.findChessboardCornersSBWithMeta(
                gray, board_size, flags=cv2.CALIB_CB_EXHAUSTIVE)
            if ok and corners is not None:
                return corners.astype(np.float32)
        except cv2.error:
            pass
    if hasattr(cv2, 'findChessboardCornersSB'):
        try:
            ok, corners = cv2.findChessboardCornersSB(gray, board_size)
            if ok and corners is not None:
                return corners.astype(np.float32)
        except cv2.error:
            pass

    # 传统方法作为回退
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
             + cv2.CALIB_CB_NORMALIZE_IMAGE
             + cv2.CALIB_CB_FAST_CHECK)
    ok, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if not ok:
        return None
    # 亚像素精化
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30, 0.001)
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return refined


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board-w', type=int, default=8,
                    help='内角点列数 (默认 8)')
    ap.add_argument('--board-h', type=int, default=6,
                    help='内角点行数 (默认 6)')
    ap.add_argument('--square', type=float, default=0.025,
                    help='方格物理边长(米), 默认 0.025')
    ap.add_argument('--frame-step', type=int, default=30,
                    help='每 N 帧采样 (默认 30)')
    ap.add_argument('--max-views', type=int, default=40,
                    help='最多收集多少视图 (默认 40)')
    ap.add_argument('--video', default=VIDEO)
    ap.add_argument('--dry-run', action='store_true',
                    help='只标定不更新 yaml')
    args = ap.parse_args()

    os.makedirs(DEBUG_DIR, exist_ok=True)
    # 清空旧 debug 图
    for f in os.listdir(DEBUG_DIR):
        if f.endswith('.jpg'):
            os.remove(os.path.join(DEBUG_DIR, f))

    if not os.path.exists(args.video):
        print(f'[ERR] 视频不存在: {args.video}')
        sys.exit(1)

    board_size = (args.board_w, args.board_h)
    print(f'=== 标定参数 ===')
    print(f'  视频: {args.video}')
    print(f'  棋盘内角点: {args.board_w} x {args.board_h}')
    print(f'  方格边长: {args.square} m')

    # 3D 物理坐标 (相机系无关, Z=0 平面)
    objp = np.zeros((args.board_h * args.board_w, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.board_w, 0:args.board_h].T.reshape(-1, 2)
    objp *= args.square

    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'  视频: {W}x{H}, {total} 帧, {fps:.1f} FPS')

    objpoints = []   # 每张图的 3D 点
    imgpoints = []   # 每张图的 2D 角点
    image_size = None

    fi = 0
    processed = 0
    found = 0
    saved_debug = 0
    print(f'\n=== 检测棋盘 (每 {args.frame_step} 帧扫一帧) ===')
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        fi += 1
        if fi % args.frame_step != 0:
            continue
        processed += 1
        if found >= args.max_views:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]   # (w, h)

        corners = detect_corners(gray, board_size)
        if corners is None:
            continue

        objpoints.append(objp)
        imgpoints.append(corners)
        found += 1

        if saved_debug < 12:
            vis = frame.copy()
            cv2.drawChessboardCorners(vis, board_size, corners, True)
            label = f'frame {fi}  view {found}'
            cv2.putText(vis, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            outp = os.path.join(DEBUG_DIR, f'v{found:02d}_f{fi:05d}.jpg')
            cv2.imwrite(outp, vis, [cv2.IMWRITE_JPEG_QUALITY, 75])
            saved_debug += 1

        if found % 5 == 0:
            print(f'  扫描帧 {fi}/{total}, 已检测到 {found} 张')

    cap.release()
    print(f'\n=== 共收集 {found} 张有效视图 (从 {processed} 帧采样中) ===')

    if found < 8:
        print(f'[ERR] 有效视图不足 ({found} < 8). 可能的原因:')
        print('  1. 棋盘内角点数错了 (当前默认 8x6，可尝试 7x5 / 9x6 / 10x7)')
        print('  2. 视频画面太模糊')
        print(f'  3. 你可以看 output/calib_debug/ 验证检测是否正确')
        sys.exit(1)

    # === OpenCV 单目标定 ===
    print(f'\n=== 调用 cv2.calibrateCamera ===')
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None)
    print(f'  RMS 重投影误差: {rms:.3f} 像素')
    print(f'  K = ')
    print(K)
    print(f'  畸变系数 (k1, k2, p1, p2, k3) = {dist.ravel()}')

    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    k1, k2, p1, p2, k3 = [float(v) for v in dist.ravel()[:5]]

    # === 输出报告 ===
    os.makedirs(os.path.dirname(REPORT_PATH), exist_ok=True)
    with open(REPORT_PATH, 'w', encoding='utf-8') as f:
        f.write(f'=== 相机标定报告 ===\n')
        f.write(f'视频: {args.video}\n')
        f.write(f'分辨率: {image_size[0]} x {image_size[1]}\n')
        f.write(f'棋盘: {args.board_w} x {args.board_h}, '
                f'方格边长 {args.square} m\n')
        f.write(f'采集视图: {found}\n')
        f.write(f'RMS 重投影误差: {rms:.3f} 像素\n')
        f.write(f'\n--- 内参 ---\n')
        f.write(f'fx = {fx:.4f}\n')
        f.write(f'fy = {fy:.4f}\n')
        f.write(f'cx = {cx:.4f}\n')
        f.write(f'cy = {cy:.4f}\n')
        f.write(f'\n--- 畸变 ---\n')
        f.write(f'k1 = {k1:.6f}\n')
        f.write(f'k2 = {k2:.6f}\n')
        f.write(f'p1 = {p1:.6f}\n')
        f.write(f'p2 = {p2:.6f}\n')
        f.write(f'k3 = {k3:.6f}\n')
    print(f'\n  报告: {REPORT_PATH}')
    print(f'  调试图: {DEBUG_DIR}  ({saved_debug} 张)')

    # === 更新 YAML ===
    if args.dry_run:
        print('\n[--dry-run] 不更新 yaml')
        return

    if os.path.exists(YAML_PATH):
        backup = YAML_PATH + '.bak'
        shutil.copy2(YAML_PATH, backup)
        print(f'\n  已备份旧 yaml: {backup}')

    yaml_content = f'''%YAML:1.0
# Industrial USB Camera Calibration ({image_size[0]}x{image_size[1]})
# Auto-generated by scripts/calibrate_camera.py
# 标定视图数: {found}, RMS={rms:.3f}px
Initializer.initialization: 1

#---------------------------------------------
# 相机参数 (OpenCV PinHole)
#---------------------------------------------
Camera.type: "PinHole"
Camera.fx: {fx:.4f}
Camera.fy: {fy:.4f}
Camera.cx: {cx:.4f}
Camera.cy: {cy:.4f}
Camera.k1: {k1:.6f}
Camera.k2: {k2:.6f}
Camera.k3: {k3:.6f}
Camera.p1: {p1:.6f}
Camera.p2: {p2:.6f}

Camera.width: {image_size[0]}
Camera.height: {image_size[1]}

# 相机帧率 (fps)
Camera.fps: 30.0

# 彩色/灰度
Camera.RGB: 1

#---------------------------------------------
# ORB 特征点参数
#---------------------------------------------
ORBextractor.nFeatures: 4000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 5
ORBextractor.minThFAST: 2

#---------------------------------------------
# 跟踪参数
#---------------------------------------------
Tracker.kfDist: 0.02
Tracker.kfDistLC: 0.05
Tracker.kfDistLC2: 0.02
Tracker.monoFOV: 20
Tracker.monoSigma: 0.04
Tracker.monoMinConf: 0.3
Tracker.monoMaxFrames: 30

#---------------------------------------------
# 优化参数
#---------------------------------------------
Optimizer.numIterations: 5
Optimizer.numIterationsIni: 5
Optimizer.numIterationsBA: 5

#---------------------------------------------
# 重载与缩放参数 (初始化需要)
#---------------------------------------------
LocalMapping.nFeatures: 4000
LocalMapping.scaleFactor: 1.2
LocalMapping.nLevels: 8

# Viewer 参数
#---------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
'''
    with open(YAML_PATH, 'w', encoding='utf-8') as f:
        f.write(yaml_content)
    print(f'  已写入: {YAML_PATH}')
    print(f'\n下一步:')
    print(f'  colcon build  (重建 orbslam3_wrapper 不是必须, 因为 yaml 是运行时读取)')


if __name__ == '__main__':
    main()
