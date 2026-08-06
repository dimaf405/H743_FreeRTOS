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
→ 平台无关 ParameterJournal
→ STM32H7 FlashPartition
```

`definitions/` 是参数生成器的集中 Schema 输入区，不是 Parameter 中间件对 Commander、RC、Rover 或 MotorOutput 业务的运行时所有权声明。文件按功能域命名并由 `make/project.mk` 显式排序；实际参数消费和生命周期仍归各自模块。集中保存可避免跨目录扫描隐式改变生成顺序，后续若改为模块就近定义，必须先证明生成元数据及持久化兼容性。

## 当前实现

- 参数数量完全由生成结果决定，不设置固定 64、128 等容量。
- 首批目录包含 24 项差速 Rover 参数：20 项 `RO_*` 和 4 项 `RD_*`。
- 官方 parser 负责解析参数定义并输出官方 XML/JSON；Header renderer 只等价实现官方模板语义，不使用正则重新解析参数源码，也不依赖 Jinja2。
- 参数生成器会确定性生成 `build/generated_include` 转发头；删除生成目录后可从源码重新构建。
- Parameter Core 提供 `param_find/get/set/reset/save/load`、稀疏 Layer、AtomicTransaction、`px4::Param<T>` 和 `ModuleParams` 兼容接口。
- TinyBSON 和 flashparams 使用调用者提供的固定或启动期 Buffer；编码/解码热路径不动态分配，不包含 fd、POSIX 或文件系统路径。
- Autosave 在首次变化后至少等待 300 ms，连续保存间隔至少 2 s，失败最多重试 3 次。
- USB CDC RX 使用固定 1024-byte SPSC Ring；ISR 不解析命令、不访问参数或 Flash、不格式化日志、不分配内存。
- 参数持久化使用 `0x081E0000～0x08200000` 单个 128 KiB 扇区追加 Journal；`ParameterJournal` 只负责 Sequence、Payload Length、CRC32、Commit Marker、扫描和回退，STM32H7 `FlashPartition` 负责 ECC 安全读、32-byte program、sector erase、回读和 cache 一致性。
- 扫描不执行整段 cache invalidate；program/erase 成功后仅由 STM32 cache helper 失效实际修改范围，D-cache 关闭时 helper 防御性 no-op。
- DBECC 只有在活动安全读窗口、地址属于参数分区且 Bank 2 标志匹配时才允许 BusFault 恢复；其他 BusFault 一律进入启动诊断和复位。
- Journal v1 字节格式、分区地址和公开返回语义保持兼容；空间不足返回 ENOSPC且不自动擦除。
- 每次 load 都重新读取并验证 Header CRC、最终 Commit Marker 和 payload CRC；缓存的最新记录失效时重新扫描整个 Journal，回退到上一条有效快照后再次复验。

## Application Runtime 生命周期

- `Param<T>` 构造只执行编译期类型约束，不调用 `param_get()` 或 `param_set_used()`；模块每次 start 必须显式 `bind()`，bind 失败时不得继续沿用上一 Runtime 的值。
- `param_shutdown()` 停止 Autosave，注销 notify/storage/lock callback，清除 ready、used、unsaved、动态 Layer、值 cache 和运行期同步对象；下一次 init 从未绑定状态开始。
- `ParameterService::shutdown()` 在释放自身 Mutex 前依次关闭 Parameter Core 和 `ParameterJournal`。Journal shutdown 清除 append/latest/snapshot 和故障缓存，下一次 initialize/load 必须重新扫描 Flash。
- Armed/Flash coordinator 仍独立于 Application Runtime，确保 ARMED 时 save/erase/confirm 延后、FlashBusy 时 Arm 被拒绝；pending 操作在 Disarm 后重试。
- 以上是源码生命周期契约。2026-08-05 的 Windows 原生 clean build、最终 ELF 和同上电 `shutdown → init → start` 目标板验收尚未完成，不沿用下方历史资源数字作为当前证明。

## 当前资源与验证

```text
.text       111020 bytes
.data         2828 bytes
.bss        325776 bytes
Signed BIN  115064 bytes
make verify PASS
```

以上仅代表当前目标构建、签名和镜像一致性验证通过。未新增或运行测试框架、测试文件、SITL 或仿真；尚未完成 USB 在线调参、自动保存、掉电恢复、CRC 损坏回退、ENOSPC 和人工擦除的实车验收。阶段 2 工作区改动已按功能拆分提交。
