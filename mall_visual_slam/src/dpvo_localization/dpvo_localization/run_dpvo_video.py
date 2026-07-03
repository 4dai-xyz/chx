from pathlib import Path
import argparse
import os
import subprocess
import sys


def _resolve_path(path, base):
    p = Path(path)
    if p.is_absolute():
        return p
    return base / p


def _strip_ros_args(argv):
    if '--ros-args' not in argv:
        return argv
    return argv[:argv.index('--ros-args')]


def _dpvo_env(root, allow_display=False):
    env = os.environ.copy()
    runtime_home = Path(os.environ.get(
        'DPVO_RUNTIME_HOME',
        '/home/ros/ros2_orbslam3/.runtime/dpvo_home',
    ))
    dpvo_pangolin_prefix = Path(os.environ.get(
        'DPVO_PANGOLIN_PREFIX',
        '/home/ros/ros2_orbslam3/.local/pangolin_dpvo_abi0',
    ))
    runtime_home.mkdir(parents=True, exist_ok=True)

    # 将 evo/matplotlib 隔离到运行时目录，避免用户桌面环境中的 TkAgg 配置影响离线运行。
    env['HOME'] = str(runtime_home)
    env['MPLBACKEND'] = 'Agg'
    env['MPLCONFIGDIR'] = str(runtime_home / '.config' / 'matplotlib')
    Path(env['MPLCONFIGDIR']).mkdir(parents=True, exist_ok=True)
    if not allow_display:
        env.pop('DISPLAY', None)
        env.pop('WAYLAND_DISPLAY', None)
    else:
        env['PANGOLIN_PREFIX'] = str(dpvo_pangolin_prefix)
        env.setdefault('DPVO_VIEWER_DISABLE_CUDA_INTEROP', '1')

    env['PYTHONPATH'] = str(root) + os.pathsep + env.get('PYTHONPATH', '')
    dpviewer_build = root / 'DPViewer' / 'build' / 'lib.linux-x86_64-cpython-310'
    if dpviewer_build.exists():
        env['PYTHONPATH'] = str(dpviewer_build) + os.pathsep + env['PYTHONPATH']
    env['CUDA_HOME'] = env.get('CUDA_HOME', '/usr/local/cuda-12.1')
    ld_library_paths = [
        '/home/ros/miniconda3/envs/dpvo/lib/python3.10/site-packages/torch/lib',
        '/home/ros/miniconda3/envs/dpvo/lib',
        '/usr/local/cuda-12.1/lib64',
        '/usr/lib/wsl/lib',
    ]
    if allow_display:
        ld_library_paths.insert(0, str(dpvo_pangolin_prefix / 'lib'))
    ld_library_paths.append(
        env.get('LD_LIBRARY_PATH', ''),
    )
    env['LD_LIBRARY_PATH'] = os.pathsep.join(ld_library_paths)
    return env


def _check_dpvo_python(dpvo_python, env):
    check_code = (
        "import torch\n"
        "print('torch', torch.__version__)\n"
        "print('torch_cuda', torch.version.cuda)\n"
        "print('cuda_available', torch.cuda.is_available())\n"
        "raise SystemExit(0 if torch.cuda.is_available() else 2)\n"
    )
    try:
        completed = subprocess.run(
            [str(dpvo_python), '-c', check_code],
            text=True,
            capture_output=True,
            check=True,
            env=env,
        )
        print(completed.stdout, end='')
    except FileNotFoundError:
        print(f'DPVO Python was not found: {dpvo_python}', file=sys.stderr)
        raise SystemExit(2)
    except subprocess.CalledProcessError as exc:
        if exc.stdout:
            print(exc.stdout, end='', file=sys.stderr)
        if exc.stderr:
            print(exc.stderr, end='', file=sys.stderr)
        print('DPVO requires a CUDA-capable GPU in the DPVO Python environment.',
              file=sys.stderr)
        raise SystemExit(2)


def _run_dpvo_enhancement(
    repo,
    root,
    dpvo_python,
    env,
    args,
    imagedir,
):
    """运行 DPVO 后处理增强模块，生成关键帧记忆和质量评估。"""
    trajectory = root / 'saved_trajectories' / f'{args.name}.txt'
    if not trajectory.exists():
        print(
            f'DPVO enhancement skipped: trajectory was not found: {trajectory}',
            file=sys.stderr,
        )
        return

    enhancement_script = repo / 'src' / 'dpvo_localization' / 'dpvo_localization' / 'dpvo_enhancement.py'
    if not enhancement_script.exists():
        print(
            f'DPVO enhancement skipped: script was not found: {enhancement_script}',
            file=sys.stderr,
        )
        return

    base_output_dir = _resolve_path(args.enhanced_output_dir, repo)
    output_dir = base_output_dir / args.name
    cmd = [
        str(dpvo_python),
        str(enhancement_script),
        '--trajectory', str(trajectory),
        '--video', str(imagedir),
        '--calib', str(args.calib),
        '--output-dir', str(output_dir),
        '--stride', str(args.stride),
        '--skip', str(args.skip),
        '--name', args.name,
        '--keyframe-max-gap', str(args.keyframe_max_gap),
        '--keyframe-min-quality', str(args.keyframe_min_quality),
        '--translation-threshold', str(args.keyframe_translation_threshold),
        '--rotation-threshold-deg', str(args.keyframe_rotation_threshold_deg),
        '--orb-features', str(args.keyframe_orb_features),
        '--relocalization-topk', str(args.relocalization_topk),
        '--preview-width', str(args.enhanced_preview_width),
        '--essential-threshold-px', str(args.essential_threshold_px),
        '--min-geometric-inliers', str(args.min_geometric_inliers),
        '--min-geometric-inlier-ratio', str(args.min_geometric_inlier_ratio),
        '--visualization-fps', str(args.enhanced_visualization_fps),
        '--visualization-every', str(args.enhanced_visualization_every),
        '--visualization-width', str(args.enhanced_visualization_width),
        '--visualization-snapshot-count', str(args.enhanced_snapshot_count),
    ]
    if args.enhanced_show:
        cmd.append('--show')
    if args.no_enhanced_video:
        cmd.append('--no-visualization-video')
    else:
        cmd.append('--save-visualization-video')
    print('Running DPVO enhancement...', flush=True)
    subprocess.run(cmd, cwd=str(repo), check=True, env=env)


def main():
    parser = argparse.ArgumentParser(description='在视频或图片目录上运行 DPVO')
    parser.add_argument('--dpvo-root', default='Opensource code/DPVO-main')
    parser.add_argument('--imagedir', default='resources/input_video.mp4_bev.mp4')
    parser.add_argument('--calib', default='Opensource code/DPVO-main/calib/custom_mall.txt')
    parser.add_argument('--name', default='mall_run')
    parser.add_argument('--stride', type=int, default=2)
    parser.add_argument('--skip', type=int, default=0)
    parser.add_argument('--viz', action='store_true')
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--save_trajectory', action='store_true')
    parser.add_argument('--save_ply', action='store_true')
    parser.add_argument('--save_colmap', action='store_true')
    parser.add_argument('--opts', nargs='*', default=[])
    parser.add_argument('--network', default='dpvo.pth')
    parser.add_argument('--no_enhance', action='store_true',
                        help='只运行原始 DPVO，不生成关键帧记忆和质量评估')
    parser.add_argument('--only_enhance', action='store_true',
                        help='不重新运行 DPVO，只分析已经保存的轨迹文件')
    parser.add_argument('--enhanced-output-dir', default='output/dpvo_enhanced',
                        help='DPVO 增强结果的基础输出目录')
    parser.add_argument('--keyframe-max-gap', type=int, default=30,
                        help='增强模块最多隔多少个 DPVO 样本插入一个关键帧')
    parser.add_argument('--keyframe-min-quality', type=float, default=0.35,
                        help='低于该质量分数的帧不进入关键帧记忆')
    parser.add_argument('--keyframe-translation-threshold', type=float, default=0.0,
                        help='关键帧平移阈值；0 表示自动估计')
    parser.add_argument('--keyframe-rotation-threshold-deg', type=float, default=0.0,
                        help='关键帧旋转阈值；0 表示自动估计')
    parser.add_argument('--keyframe-orb-features', type=int, default=1200,
                        help='每个关键帧保存多少个 ORB 特征')
    parser.add_argument('--relocalization-topk', type=int, default=5,
                        help='每个低质量片段保留几个重定位候选关键帧')
    parser.add_argument('--enhanced-preview-width', type=int, default=960,
                        help='增强模块保存关键帧图像时的最大宽度')
    parser.add_argument('--essential-threshold-px', type=float, default=1.5,
                        help='第二版几何重定位验证的 Essential Matrix RANSAC 像素阈值')
    parser.add_argument('--min-geometric-inliers', type=int, default=35,
                        help='第二版几何重定位验证通过所需的最少 RANSAC 内点数')
    parser.add_argument('--min-geometric-inlier-ratio', type=float, default=0.22,
                        help='第二版几何重定位验证通过所需的最小内点比例')
    parser.add_argument('--enhanced-show', action='store_true',
                        help='增强后处理时实时显示可视化窗口，按 q 或 Esc 退出')
    parser.add_argument('--no-enhanced-video', action='store_true',
                        help='不保存增强可视化 MP4 视频')
    parser.add_argument('--enhanced-visualization-fps', type=float, default=0.0,
                        help='增强可视化视频帧率；0 表示按视频 FPS/stride 自动估计')
    parser.add_argument('--enhanced-visualization-every', type=int, default=1,
                        help='增强可视化每隔多少个 DPVO 样本写一帧')
    parser.add_argument('--enhanced-visualization-width', type=int, default=1440,
                        help='增强可视化视频最大宽度')
    parser.add_argument('--enhanced-snapshot-count', type=int, default=12,
                        help='增强可视化额外保存多少张截图')
    args = parser.parse_args(_strip_ros_args(sys.argv[1:]))

    repo = Path('/home/ros/ros2_orbslam3')
    root = _resolve_path(args.dpvo_root, repo)
    demo = root / 'demo.py'
    imagedir = _resolve_path(args.imagedir, repo)
    calib = _resolve_path(args.calib, repo)
    network = _resolve_path(args.network, root)

    missing = [p for p in [demo, imagedir, calib, network] if not p.exists()]
    if missing:
        print('DPVO run prerequisites are missing:', file=sys.stderr)
        for p in missing:
            print(f'  - {p}', file=sys.stderr)
        raise SystemExit(2)

    dpvo_python = Path(os.environ.get(
        'DPVO_PYTHON',
        '/home/ros/miniconda3/envs/dpvo/bin/python',
    ))
    env = _dpvo_env(root, allow_display=args.viz or args.enhanced_show)
    if not args.only_enhance:
        _check_dpvo_python(dpvo_python, env)

    enhance_enabled = not args.no_enhance
    if enhance_enabled:
        args.save_trajectory = True

    cmd = [
        str(dpvo_python),
        str(demo),
        '--imagedir', str(imagedir),
        '--calib', str(calib),
        '--name', args.name,
        '--stride', str(args.stride),
        '--skip', str(args.skip),
        '--network', str(network),
    ]
    if args.viz:
        cmd.append('--viz')
    if args.plot:
        cmd.append('--plot')
    if args.save_trajectory:
        cmd.append('--save_trajectory')
    if args.save_ply:
        cmd.append('--save_ply')
    if args.save_colmap:
        cmd.append('--save_colmap')
    if args.opts:
        cmd.append('--opts')
        cmd.extend(args.opts)

    if not args.only_enhance:
        subprocess.run(cmd, cwd=str(root), check=True, env=env)

    if enhance_enabled:
        _run_dpvo_enhancement(repo, root, dpvo_python, env, args, imagedir)


if __name__ == '__main__':
    main()
