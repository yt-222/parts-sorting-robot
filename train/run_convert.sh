#!/bin/bash
# ==============================================================================
# 在 Docker 容器内运行的 Kmodel 量化编译命令
# 容器工作目录: /workspace/test_yolov8/detect
# ==============================================================================

cd /workspace/test_yolov8/detect

# 执行量化命令：320x320 输入，Squant 与高精度量化
python to_kmodel.py --target k230 --model best.onnx --dataset calib --input_width 320 --input_height 320 --ptq_option 0

echo "量化编译结束，请在宿主机目录下检查是否已生成 best.kmodel！"
