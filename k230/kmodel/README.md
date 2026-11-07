# K230 YOLOv8 模型部署说明

本目录下的 `best.kmodel` 为最终板端推理模型（nncase INT8 量化格式），可直接拷贝到 TF/MicroSD 卡挂载到 K230 设备的 `/sdcard/kmodel/best.kmodel` 使用。

> 若需重新训练生成模型，请参考根目录 `train/` 下的训练与量化指南（Roboflow 标注 → 本地 YOLOv8 训练 → NNCase 量化编译）。

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
