# Dima Parameter 中间件

## 阶段 2 状态

- PX4 来源：v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
- 状态：Parameter 基础链代码与目标构建验证完成；实车验收待完成。
- 许可证：`PENDING`；最终处理 `DEFERRED`，不阻塞当前移植和调试。

## 模块链

```text
PX4 PARAM_DEFINE_* 参数定义
→ 官方 process parser / scanner
→ parameters.xml + parameters.json
→ px4_parameters.hpp + parameter_metadata.c
→ Parameter Layer/Core + AtomicTransaction
→ px4::Param<T> / ModuleParams / parameter_update
→ Autosave / USB Parameter Command Service
→ TinyBSON flashparams Buffer
→ 单扇区 Flash Journal
```

## 当前实现

- 参数数量完全由生成结果决定，不设置固定 64、128 等容量。
- 首批目录包含 24 项差速 Rover 参数：20 项 `RO_*` 和 4 项 `RD_*`。
- 官方 parser 负责解析参数定义并输出官方 XML/JSON；Header renderer 只等价实现官方模板语义，不使用正则重新解析参数源码，也不依赖 Jinja2。
- 参数生成器会确定性生成 `build/generated_include` 转发头；删除生成目录后可从源码重新构建。
- Parameter Core 提供 `param_find/get/set/reset/save/load`、稀疏 Layer、AtomicTransaction、`px4::Param<T>` 和 `ModuleParams` 兼容接口。
- TinyBSON 和 flashparams 使用调用者提供的固定或启动期 Buffer；编码/解码热路径不动态分配，不包含 fd、POSIX 或文件系统路径。
- Autosave 在首次变化后至少等待 300 ms，连续保存间隔至少 2 s，失败最多重试 3 次。
- USB CDC RX 使用固定 1024-byte SPSC Ring；ISR 不解析命令、不访问参数或 Flash、不格式化日志、不分配内存。
- 参数持久化使用 `0x081E0000～0x08200000` 单个 128 KiB 扇区追加 Journal；记录包含 Sequence、Payload Length、CRC32 和最终 Commit Marker；Bank 2 扫描使用 ECC 安全读，DBECC 仅在参数区受控恢复并跳过不可读记录；空间不足返回 ENOSPC且不自动擦除。
- 每次 load 都重新读取并验证 Header CRC、最终 Commit Marker 和 payload CRC；缓存的最新记录失效时重新扫描整个 Journal，回退到上一条有效快照后再次复验。

## 当前资源与验证

```text
.text       111020 bytes
.data         2828 bytes
.bss        325776 bytes
Signed BIN  115064 bytes
make verify PASS
```

以上仅代表当前目标构建、签名和镜像一致性验证通过。未新增或运行测试框架、测试文件、SITL 或仿真；尚未完成 USB 在线调参、自动保存、掉电恢复、CRC 损坏回退、ENOSPC 和人工擦除的实车验收。阶段 2 工作区改动已按功能拆分提交。
