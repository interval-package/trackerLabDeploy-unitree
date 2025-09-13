#!/bin/bash
set -e

# 激活 conda 环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate jetson38

# 安装依赖
sudo apt-get update
sudo apt-get install -y libopenblas-base libopenmpi-dev libomp-dev

# 下载并安装 PyTorch (官方 wheel, Python 3.8, JetPack 5.1.1)
TORCH_WHL="torch-2.0.0a0+8cbe2d7.nv23.05-cp38-cp38-linux_aarch64.whl"
wget https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/${TORCH_WHL}
pip install --no-cache-dir ${TORCH_WHL}

# 安装 torchvision (源码方式)
git clone --branch v0.15.2 https://github.com/pytorch/vision.git
cd vision
python setup.py install
cd ..

# 测试
python - <<'EOF'
import torch, torchvision
print("Torch version:", torch.__version__)
print("Torchvision version:", torchvision.__version__)
print("CUDA available:", torch.cuda.is_available())
print("CUDA device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "N/A")
EOF
