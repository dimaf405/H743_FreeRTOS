# FatFs R0.12c integration

- 来源：`STM32Cube_FW_H7_V1.10.0` 中的 FatFs R0.12c。
- 未改核心：`src/ff.c`、`src/ff.h`、`src/integer.h`。
- 本地配置与 disk ABI：`src/ffconf.h`、`src/diskio.h`。参数文件使用 ASCII 8.3 路径，不启用 LFN 或字符集转换表。
- `FileStorage` 的后端互斥量串行化唯一 FatFs 客户端，因此 FatFs 自身不再创建重复的同步对象。
- H743 SDMMC 实现：`Boards/H743/Src/fatfs_diskio.c`。
- 参数文件 capability 实现：`Dima/platform/freertos/storage/FatFsParameterFileStore.cpp`。

FatFs 核心文件保留 ChaN 原始许可文字；STM32Cube 包装与本地适配的最终许可证状态仍为
`PENDING`，详见 `docs/DIMA_SOURCE_MANIFEST.md`，发布前必须从原始软件包补齐准确的
Package License/Notice，不以本 README 代替许可证。
