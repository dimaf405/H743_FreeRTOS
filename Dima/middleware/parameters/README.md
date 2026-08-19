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
→ Autosave / MAVLink Parameter Protocol
→ TinyBSON flashparams Buffer
→ 平台无关 ParameterJournal
→ STM32H7 FlashPartition
```

`definitions/` 是固定功能参数的集中 Schema 输入区，不是 Parameter 中间件对 Commander、RC、Rover 或 MotorOutput 业务的运行时所有权声明。板级串口例外地由 `Boards/H743/serial_ports.json` 通过 `tools/serial/generate_config.py` 生成，避免参数、Runtime 和引脚表各写一份。全部输入仍由 `make/project.mk` 显式列出；实际参数消费和生命周期归各自模块。

## 当前实现

- 参数数量完全由生成结果决定，不设置固定 64、128 等容量。
- 当前 Schema 共 205 项。`qgc_compat_params.c` 只保留 11 项固定 QGC 身份、校准摘要与失联策略兼容合同；未实现的 `COM_FLTMODE1..6` 与 `RTL_*` 不进入固件或 Metadata。`Boards/H743/serial_ports.json` 另行生成八组 `SERIAL1..8_BAUD/FUNCTION` 和内部 `DIMA_SER_VER`。串口编号直接对应 VCU-H7 板载 USART/UART1..8，Function 当前只公开 Disabled/RC Input；CAL/低电量条目只是固定兼容值，不代表相关未来能力已实现。
- 首批目录包含 24 项差速 Rover 参数：20 项 `RO_*` 和 4 项 `RD_*`。
- 官方 parser 只扫描 `make/project.mk` 显式列出的定义文件并输出 XML/JSON；Header renderer 只等价实现官方模板语义，不依赖 Jinja2。原 178 项 handle 顺序由 R331 基线哈希固定；11 项 QGC 合同与 17 项板级串口状态组成 28 项 stable tail，使 `SYS_AUTOSTART` 保持原 index 177，其余 27 项只追加到末尾。Header、类型表、`param_info` 和 JSON 使用同一顺序。
- Component Metadata 生成器从同一 JSON 构造 QGC version 1 公开副本，过滤 `RC_PORT_CONFIG`/`DIMA_SER_VER` 后保留 203 项描述，XZ 压缩并计算 PX4 CRC；General JSON 只声明 Parameter type。两份 XZ 作为只读 Flash 数组由 MavlinkService/FTP 交付，Parameter Core 不感知传输层。
- 参数生成器会确定性生成 `build/generated_include` 转发头；删除生成目录后可从源码重新构建。
- Parameter Core 的运行期状态、事务及 get/set/reset 位于 `param.cpp`；持久化后端注册、save/load/status/erase 位于 `param_storage.cpp`，两者仅通过私有 `param_internal.hpp` 共享状态。公开兼容接口仍统一由 `param.h` 提供。
- TinyBSON 和 flashparams 使用调用者提供的固定或启动期 Buffer；编码/解码热路径不动态分配，不包含 fd、POSIX 或文件系统路径。
- Autosave 在首次变化后至少等待 300 ms，连续保存间隔至少 2 s，失败最多重试 3 次。
- ParameterService 不持有或读取 Console，只负责 Core、Journal、Autosave 和 Flash 事务；在线参数由 MavlinkService 通过 Classic/Ext 协议访问。
- MavlinkService 在每次 LIST 开始前一次性激活全部 QGC 固定参数及 16 项公开 `SERIALx_BAUD/FUNCTION`，并在参数锁内冻结 used 句柄快照；整轮 PARAM_VALUE、按 index 缺帧补读以及快照内参数的 READ/SET 回包都复用该 count/index。`RC_PORT_CONFIG` 与 `DIMA_SER_VER` 只服务迁移，不进入 LIST。固定参数仅允许同值写回；串口 Function 只允许 Disabled/RC Input，并拒绝产生多个 RC owner。
- 参数持久化使用 `0x081E0000～0x08200000` 单个 128 KiB 扇区追加 Journal；`ParameterJournal` 只负责 Sequence、Payload Length、CRC32、Commit Marker、扫描和回退，STM32H7 `FlashPartition` 负责 ECC 安全读、32-byte program、sector erase、回读和 cache 一致性。
- 扫描不执行整段 cache invalidate；program/erase 成功后仅由 STM32 cache helper 失效实际修改范围，D-cache 关闭时 helper 防御性 no-op。
- DBECC 只有在活动安全读窗口、地址属于参数分区且 Bank 2 标志匹配时才允许 BusFault 恢复；其他 BusFault 一律进入启动诊断和复位。
- Journal v1 字节格式、分区地址和公开返回语义保持兼容；空间不足返回 ENOSPC且不自动擦除。
- Journal 解码边界忽略 11 项 QGC 固定合同以及兼容占位 `RC_MAP_FLTMODE` 的旧存储值；16 项 `SERIALx_BAUD/FUNCTION` 与其他可配置参数一样恢复并由 Autosave 持久化。Schema v1 的七组旧参数先以旧默认值补全，再按物理 UART 迁移到直接编号的 v2；旧 `RC_PORT_CONFIG` 和旧功能命名 baud 值只在 Schema v0 时迁移一次，未来版本在降级时 fail-closed。
- 每次 load 都重新读取并验证 Header CRC、最终 Commit Marker 和 payload CRC；缓存的最新记录失效时重新扫描整个 Journal，回退到上一条有效快照后再次复验。

## Application Runtime 生命周期

- `Param<T>` 构造只执行编译期类型约束，不调用 `param_get()` 或 `param_set_used()`；模块每次 start 必须显式 `bind()`，bind 失败时不得继续沿用上一 Runtime 的值。
- `param_shutdown()` 停止 Autosave，注销 notify/storage/lock callback，清除 ready、used、unsaved、动态 Layer、值 cache 和运行期同步对象；下一次 init 从未绑定状态开始。
- `ParameterService::shutdown()` 在释放自身 Mutex 前依次关闭 Parameter Core 和 `ParameterJournal`。Journal shutdown 清除 append/latest/snapshot 和故障缓存，下一次 initialize/load 必须重新扫描 Flash。
- Armed/Flash coordinator 仍独立于 Application Runtime，确保 ARMED 时 save/erase/confirm 延后、FlashBusy 时 Arm 被拒绝；pending 操作在 Disarm 后重试。
- 以上是源码生命周期契约。同上电 `shutdown → init → start`、掉电恢复、损坏回退和 ENOSPC 仍需目标板验收，不能由源码或主机生成结果代替。

## 当前验证边界

参数生成链当前确定性产出 205 项固件参数和 203 项公开 Metadata；关键 stable-tail 索引、JSON/XZ 与 CRC 由正式生成链统一给出。整机 `.text/.data/.bss`、签名镜像和 Factory HEX 属于全项目构建结果，只在 `docs/DIMA_SOURCE_MANIFEST.md` 记录最新 Windows 原生 clean build，避免本模块 README 复制一份易陈旧的资源数字。未新增或运行测试框架、测试文件、SITL 或仿真；MAVLink 在线调参、自动保存、掉电恢复、CRC 损坏回退和 ENOSPC 仍为实板验收项。文本控制台和人工存储擦除不属于当前 Runtime 接口。
