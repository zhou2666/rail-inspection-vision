# 模型文件

| 文件 | 大小 | 已确认用途 |
| --- | --- | --- |
| `1best.kmodel` | 3,362,480 字节 | `rail_k230.py` 当前引用的板端模型，部署到 `/sdcard/1best.kmodel`。 |
| `1best.onnx` | 10,447,687 字节 | 原材料中的导出模型，可读取其输入输出与类别元数据；没有转换记录证明它与同名 KModel 的精确对应关系。 |
| `best.kmodel` | 3,353,192 字节 | 原材料中的另一份模型，主程序未选用；用途与类别尚未确认。 |

`1best.onnx` 元数据记录：Ultralytics YOLOv12n，导出工具版本 8.3.163，任务 `detect`；输入 `images` 为 `[1, 3, 320, 320]`，输出 `output0` 为 `[1, 12, 2100]`。8 个类别名称与检测程序一致。详情见 [提取结果](../docs/MODEL_METADATA.json)。

这些是对本地 ONNX 文件的结构读取，不是模型精度、完整性或 KModel 推理兼容性的验证。字段解读依据 [ONNX 官方协议定义](https://github.com/onnx/onnx/blob/main/onnx/onnx.proto)。

模型元数据的 `license` 字段标注 `AGPL-3.0 License (https://ultralytics.com/license)`。这里保留该来源信息，不将其覆盖为 MIT 或其他自选许可；对应源码、训练权重、数据来源及完整发布要求仍需原作者核对。另两份 KModel 不因同目录存放就自动获得相同授权结论。
