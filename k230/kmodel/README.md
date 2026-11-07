# K230 YOLOv8 模型部署说明

请将训练好并转换为 nncase 格式的 `.kmodel` 文件重命名为 `best.kmodel`，放置于本目录下，并通过 TF/MicroSD 卡挂载到 K230 设备的 `/sdcard/kmodel/best.kmodel`。

> 因 `.kmodel` 为二进制权重文件（体积较大），本项目仓库不直接包含该文件。请参考根目录 `train/` 下的训练与量化指南，从你自己的 Roboflow 数据集重新训练并编译生成。

## 模型类别严格约束

模型需包含以下三类标签，且输出通道顺序必须对应：

| 索引 | 标签 | 说明 |
| :---: | :--- | :--- |
| 0 | `cube` | 正方体 |
| 1 | `cylinder` | 圆柱体 |
| 2 | `prism` | 正三棱柱 |

## 模型规格

- 输入分辨率：`320 × 320`
- 量化方式：NNCase INT8 (PTQ, `--ptq_option 0`)
- 推理输出：`[boxes, class_ids, scores]`
