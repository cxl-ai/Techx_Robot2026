#!/bin/bash
# ============================================================
# TECHx_vision — Jetson Orin NX 一键环境安装脚本
# ============================================================
# 适用于 JetPack 6 (R36) + CUDA 12.6
# 在新的 Jetson 设备上运行此脚本即可完成全部环境配置
# ============================================================
set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  TECHx_vision Jetson 环境安装${NC}"
echo -e "${CYAN}========================================${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── [1/5] 创建 conda 环境 ──
echo -e "\n${YELLOW}[1/5] 创建 Python 3.10 conda 环境...${NC}"
if conda env list 2>/dev/null | grep -q "^techx "; then
    echo -e "${GREEN}  conda 环境 'techx' 已存在，跳过创建${NC}"
else
    conda create -n techx python=3.10 -y
    echo -e "${GREEN}  conda 环境 'techx' 创建完成${NC}"
fi

eval "$(conda shell.bash hook)"
conda activate techx

# ── [2/5] 安装核心依赖 ──
echo -e "\n${YELLOW}[2/5] 安装核心 Python 依赖...${NC}"
pip install ultralytics opencv-python numpy Pillow onnxruntime pyserial 2>&1 | tail -3

# ── [3/5] 安装 Orbbec 深度相机 SDK ──
echo -e "\n${YELLOW}[3/5] 安装 Orbbec Gemini 335L SDK...${NC}"
ORBBEC_WHEEL="$SCRIPT_DIR/wheels/linux-aarch64/pyorbbecsdk2-2.1.1-cp310-cp310-manylinux_2_27_aarch64.whl"
if [ -f "$ORBBEC_WHEEL" ]; then
    pip install "$ORBBEC_WHEEL" 2>&1 | tail -3
    echo -e "${GREEN}  Orbbec SDK 安装完成${NC}"
else
    echo -e "${RED}  警告: 找不到 $ORBBEC_WHEEL${NC}"
    echo -e "${YELLOW}  深度相机将不可用，仅能使用 UVC 模式${NC}"
fi

# ── [4/5] 安装 Jetson GPU 加速 (PyTorch + torchvision) ──
echo -e "\n${YELLOW}[4/5] 安装 Jetson GPU 加速...${NC}"

# NVIDIA Jetson PyTorch (CUDA 12.6 原生)
TORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl"
echo -e "  安装 NVIDIA Jetson PyTorch 2.5..."
pip install "$TORCH_URL" --no-deps 2>&1 | tail -3

# torchvision (兼容补丁由 main.py 自动应用)
echo -e "  安装 torchvision..."
pip install torchvision==0.20.0 --no-deps 2>&1 | tail -3

# 验证 GPU
echo -e "  验证 GPU..."
python -c "
import torch
if torch.cuda.is_available():
    print(f'  GPU: {torch.cuda.get_device_name(0)} | CUDA {torch.version.cuda}')
else:
    print('  WARNING: GPU 不可用，将回退 CPU')
"

# ── [5/5] 模型文件检查 ──
echo -e "\n${YELLOW}[5/5] 检查模型文件...${NC}"
MODEL_DIR="$SCRIPT_DIR/models"
if [ -d "$MODEL_DIR" ]; then
    for d in "$MODEL_DIR"/*/; do
        name=$(basename "$d")
        has_pt=$(ls "$d"/*.pt 2>/dev/null | wc -l)
        has_onnx=$(ls "$d"/*.onnx 2>/dev/null | wc -l)
        echo -e "  ${name}: .pt=$has_pt .onnx=$has_onnx"
    done
else
    echo -e "${YELLOW}  models/ 目录不存在，请单独拷贝模型文件${NC}"
fi

# ── 完成 ──
echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}  TECHx_vision 环境安装完成!${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "  启动命令: ${CYAN}./start_jetson.sh${NC}"
echo -e "  配置文件: ${CYAN}config.json${NC} (修改 GMK IP 地址)"
echo -e "  日志目录: ${CYAN}logs/${NC}"
