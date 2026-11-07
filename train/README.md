# 模型训练与量化部署指南

本目录记录了「零件分拣机器人」视觉模型的完整训练链路：**Roboflow 网页标注 → 本地 YOLOv8 训练 → Docker/NNCase 量化编译 → 板端部署**。

---

## 一、数据标注（Roboflow 网页端）

1. 将拍摄好的工件图片上传至 [Roboflow](https://roboflow.com)。
2. 模式选择 `Object Detection`（Bounding Boxes）。
3. **顶面标注法则**：严格只框出工件的顶面平面几何图元（正方形 / 圆形 / 三角形），矩形框像素级贴合外边缘，彻底忽略侧面及投影。
   - 这样检测框几何中心恒等于垂直抓取的轴心坐标，解决「中心点漂移」问题。
4. 预处理启用 Resize 至 `320×320`，按 Train / Valid / Test 划分。
5. 导出为 **YOLOv8** 格式压缩包并解压到本地，目录规范命名为 `dataset_black_mat`（**注意：单下划线**）。

## 二、本地训练与导出（Windows + Miniconda）

```bash
# 1. 激活本地环境（Python 3.9）
conda activate k230_yolo

# 2. 安装依赖
pip install -r requirements.txt

# 3. 进入工作目录
cd D:\K230_Convert

# 4. 本地启动 YOLOv8n 训练 (100 Epochs, 输入尺寸 320)
yolo task=detect mode=train model=yolov8n.pt data=D:/K230_Convert/dataset_black_mat/data.yaml epochs=100 imgsz=320

# 5. 本地导出为 ONNX 模型
yolo task=detect mode=export model=runs/detect/train/weights/best.pt format=onnx imgsz=320
```

## 三、Docker 量化编译生成 Kmodel

将生成的 `best.onnx` 以及挑出的 **60 张黑底校准图**（放在 `calib` 文件夹）拷贝至 Docker 挂载目录，进入容器命令行：

```bash
# 注意全小写 cd 命令，不要加冒号
cd /workspace/test_yolov8/detect

# 执行量化转换（PTQ 选项 0：高精度量化）
python to_kmodel.py --target k230 --model best.onnx --dataset calib --input_width 320 --input_height 320 --ptq_option 0
```

编译成功后，在宿主机目录提取生成的 `best.kmodel`。仓库内也提供了一键脚本 `run_convert.sh` 可直接在容器内执行。

## 四、板端部署

1. 将 `best.kmodel` 拷贝到 TF 卡的 `/kmodel/best.kmodel`。
2. 将 `../k230/main.py` 放到 SD 卡根目录（如需上电自启可另存为 `main.py`）。
3. TF 卡插回 K230 并开机，打开 CanMV IDE 连接板卡运行。
4. 串口以 115200 波特率向外输出：`{class_id},{cx},{cy},{pixel_ratio}\n`。

---

## 常见报错排查

| 报错 | 原因与解决 |
| :--- | :--- |
| `FileNotFoundError: '.../data.yaml' does not exist` | 检查解压是否多套一层子目录，或文件夹名是否误拼成双下划线 `dataset__black_mat` |
| `CD: not found` / `can't cd to :` | Docker/Linux 终端切勿输入大写 `CD`，路径后不可带冒号 `:` |
| `ImportError: no module named 'sensor'` | 厂商定制固件必须使用 `from libs.PipeLine import PipeLine` 架构，不可使用未导出的原生 `sensor` 模块 |

## 参考链接

- [Ultralytics 官方文档](https://docs.ultralytics.com)
- [Canaan Kendryte nncase 编译器](https://github.com/kendryte/nncase)
- [K230 CanMV 文档仓库](https://github.com/kendryte/k230_canmv_docs)
