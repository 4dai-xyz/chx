import os
import shutil
import subprocess
import sys
from pathlib import Path


def _run_dpvo_python_check(dpvo_python):
    check_code = r"""
import importlib.util
import torch

print(f"OK   torch: {torch.__version__}")
print(f"INFO torch CUDA build: {torch.version.cuda}")
cuda_ok = torch.cuda.is_available()
print(f"{'OK  ' if cuda_ok else 'MISS'} torch.cuda.is_available(): {cuda_ok}")
if cuda_ok:
    print(f"OK   GPU: {torch.cuda.get_device_name(0)}")

modules_ok = True
for module in [
    'dpvo',
    'cuda_corr',
    'cuda_ba',
    'lietorch_backends',
    'evo',
    'yacs',
    'cv2',
    'numpy',
]:
    ok = importlib.util.find_spec(module) is not None
    print(f"{'OK  ' if ok else 'MISS'} python module: {module}")
    modules_ok = modules_ok and ok

raise SystemExit(0 if cuda_ok and modules_ok else 2)
"""
    env = os.environ.copy()
    dpvo_root = '/home/ros/ros2_orbslam3/Opensource code/DPVO-main'
    env['PYTHONPATH'] = dpvo_root + os.pathsep + env.get('PYTHONPATH', '')
    env['CUDA_HOME'] = env.get('CUDA_HOME', '/usr/local/cuda-12.1')
    env['LD_LIBRARY_PATH'] = os.pathsep.join([
        '/home/ros/miniconda3/envs/dpvo/lib/python3.10/site-packages/torch/lib',
        '/home/ros/miniconda3/envs/dpvo/lib',
        '/usr/local/cuda-12.1/lib64',
        '/usr/lib/wsl/lib',
        env.get('LD_LIBRARY_PATH', ''),
    ])

    print(f"INFO DPVO_PYTHON: {dpvo_python}")
    try:
        completed = subprocess.run(
            [dpvo_python, '-c', check_code],
            text=True,
            capture_output=True,
            env=env,
            check=True,
        )
        print(completed.stdout, end='')
        return True
    except FileNotFoundError:
        print(f"MISS DPVO_PYTHON not found: {dpvo_python}")
        return False
    except subprocess.CalledProcessError as exc:
        if exc.stdout:
            print(exc.stdout, end='')
        if exc.stderr:
            print(exc.stderr, end='', file=sys.stderr)
        return False


def main():
    ok = True

    print("== GPU / CUDA ==")
    nvidia_smi = shutil.which('nvidia-smi')
    nvcc = shutil.which('nvcc')
    print(f"{'OK  ' if nvidia_smi else 'MISS'} nvidia-smi: {nvidia_smi or 'not found'}")
    print(f"{'OK  ' if nvcc else 'MISS'} nvcc: {nvcc or 'not found'}")
    print(f"INFO CUDA_HOME: {os.environ.get('CUDA_HOME') or 'not set'}")

    print("\n== DPVO Python environment ==")
    dpvo_python = os.environ.get(
        'DPVO_PYTHON',
        '/home/ros/miniconda3/envs/dpvo/bin/python',
    )
    ok = _run_dpvo_python_check(dpvo_python) and ok

    print("\n== Files ==")
    repo = Path('/home/ros/ros2_orbslam3')
    checks = [
        repo / 'Opensource code/DPVO-main/demo.py',
        repo / 'Opensource code/DPVO-main/calib/custom_mall.txt',
        repo / 'Opensource code/DPVO-main/dpvo.pth',
        repo / 'resources/input_video.mp4_bev.mp4',
    ]
    for path in checks:
        exists = path.exists()
        print(f"{'OK  ' if exists else 'MISS'} {path}")
        ok = ok and exists

    print("\n== Result ==")
    if ok:
        print("DPVO environment looks ready.")
    else:
        print("DPVO environment is not ready. See missing items above.")
        raise SystemExit(2)


if __name__ == '__main__':
    main()
