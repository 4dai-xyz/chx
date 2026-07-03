#!/usr/bin/env bash
# 打开 VGGT + COLMAP + 3DGS 训练后的 Gaussian Splatting 渲染查看器。
# 这个脚本读取的是 gsplat 的 checkpoint，不是普通点云 PLY；
# 因此浏览器里看到的是 3D Gaussian Splatting 实时渲染效果。

set -euo pipefail

REPO_DIR="/home/ros/ros2_orbslam3"
CONDA_PYTHON="/home/ros/miniconda3/envs/dpvo/bin/python"
GSPLAT_ROOT="${REPO_DIR}/thirdparty/gsplat"
GSPLAT_EXAMPLES="${GSPLAT_ROOT}/examples"
RESULT_DIR="${REPO_DIR}/output/vggt_input_video_show/gsplat_results"
CKPT_PATH="${RESULT_DIR}/ckpts/ckpt_29999_rank0.pt"
PORT="${1:-8091}"
EXTRA_ARGS=("${@:2}")

if [ ! -f "${CKPT_PATH}" ]; then
  echo "找不到 3DGS checkpoint: ${CKPT_PATH}" >&2
  echo "请先确认 3DGS 训练已经正常结束。" >&2
  exit 1
fi

export MPLCONFIGDIR="/tmp/matplotlib_${USER:-ros}"
export TORCH_EXTENSIONS_DIR="/tmp/torch_extensions_${USER:-ros}_cccl129"
export TORCH_CUDA_ARCH_LIST="8.9"
export MAX_JOBS="1"

# 只启用标准 3DGS 渲染链路，避免加载当前项目不需要的 2DGS / 3DGUT / Adam 扩展。
export BUILD_3DGS="1"
export BUILD_2DGS="0"
export BUILD_3DGUT="0"
export BUILD_ADAM="0"
export BUILD_RELOC="0"
export BUILD_CAMERA_WRAPPERS="0"

# 这里只加入 gsplat 源码和 examples。
# 不加入 thirdparty/python_packages，因为其中有一个早期的 nerfview 兼容壳；
# 那个兼容壳只能让程序不报错，不能真正把 3DGS 渲染结果推送到网页。
# 官方 nerfview 已安装在 conda 环境的 site-packages 中，应由 Python 正常导入。
export PYTHONPATH="${GSPLAT_ROOT}:${GSPLAT_EXAMPLES}:${PYTHONPATH:-}"

mkdir -p "${MPLCONFIGDIR}" "${TORCH_EXTENSIONS_DIR}"

cd "${GSPLAT_EXAMPLES}"
exec "${CONDA_PYTHON}" simple_viewer.py \
  --ckpt "${CKPT_PATH}" \
  --output_dir "${RESULT_DIR}" \
  --port "${PORT}" \
  "${EXTRA_ARGS[@]}"
