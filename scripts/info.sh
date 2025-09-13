echo "==== OS Release ===="
cat /etc/os-release

echo "==== L4T / JetPack ===="
dpkg -l | grep nvidia-jetpack || echo "jetpack not found"
dpkg -l | grep nvidia-l4t-core || echo "l4t-core not found"

echo "==== CUDA version ===="
nvcc --version || echo "nvcc not found"
cat /usr/local/cuda/version.txt 2>/dev/null || echo "no cuda version.txt"

echo "==== cuDNN version ===="
dpkg -l | grep nvinfer | grep runtime || echo "no tensorrt runtime"
dpkg -l | grep cudnn || echo "no cudnn found"

echo "==== Python ===="
which python
python --version

echo "==== Pip ===="
which pip
pip --version

echo "==== Conda Env ===="
conda info --envs
conda list | grep python

echo "==== ENV VARS ===="
echo "PATH=$PATH"
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
echo "PYTHONPATH=$PYTHONPATH"
