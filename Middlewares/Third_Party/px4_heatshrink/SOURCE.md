# PX4 heatshrink 来源

- 上游仓库：<https://github.com/PX4/heatshrink.git>
- 固定 commit：`052e6de72f67f1777198bce98f3de62f7f3c16a0`
- 许可证：BSD-3-Clause，见 `LICENSE`

本目录仅同步固件运行期解码压缩 uORB 字段合同所需的 decoder 闭包，不导入
encoder、benchmark 或上游测试。同步的源码保持逐字一致；产品只通过
`uORBMessageFields` 使用它，不维护第二份 Topic、字段或 codec 清单。

生成阶段使用的 Python encoder 位于
`tools/upstream/uorb_v1_17/src/lib/heatshrink/heatshrink_encode.py`，其来源与
SHA-256 由同目录 `SOURCE_MANIFEST.json` 自动校验。
