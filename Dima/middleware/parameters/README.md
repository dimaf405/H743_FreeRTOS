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
→ 平台无关 FlashFS（Flash）/ FileStorage（SD 卡策略）
→ STM32H7 FlashPartition / ParameterFileStore capability
→ FreeRTOS FatFs file backend / Boards H743 SDMMC disk port
```

`definitions/` 是当前固件参数目录的集中输入区，不是 Parameter 中间件对 Commander、RC、Rover 或 MotorOutput 业务的运行时所有权声明。板级串口例外地由 `Boards/H743/serial_ports.json` 通过 `tools/serial/generate_config.py` 生成，避免参数、Runtime 和引脚表各写一份。全部输入仍由 `make/project.mk` 显式列出；实际参数消费和生命周期归各自模块。

## 当前实现

- 参数数量完全由生成结果决定，不设置固定 64、128 等容量。
- 当前固件和公开 Parameter Metadata 均为 203 项。`qgc_compat_params.c` 只保留 11 项 QGC 身份、校准摘要与失联策略合同；未实现的 `COM_FLTMODE1..6`、`RTL_*`、旧板级键和迁移版本参数不进入固件或 Metadata。`Boards/H743/serial_ports.json` 另行生成八组 `SERIAL1..8_BAUD/FUNCTION`。串口编号直接对应 VCU-H7 板载 USART/UART1..8，Function 当前只公开 Disabled/RC Input；CAL/低电量条目只是固定外部兼容值，不代表相关未来能力已实现。
- 首批目录包含 24 项差速 Rover 参数：20 项 `RO_*` 和 4 项 `RD_*`。
- 官方 parser 只扫描 `make/project.mk` 显式列出的定义文件并输出 XML/JSON；Header renderer 只等价实现官方模板语义，不依赖 Jinja2。参数按当前目录名称排序，不保留旧固件 handle 或 stable-tail 排序；Header、类型表、`param_info` 和 JSON 使用同一顺序。
- Component Metadata 生成器从同一 JSON 构造含全部 203 项参数的 QGC version 1 Parameter 文件和六路 output-only Actuator 文件，XZ 压缩并计算 PX4 CRC；General JSON 声明 Parameter type 1 与 Actuator type 5。三份 XZ 作为只读 Flash 数组由 MavlinkService/FTP 交付，Parameter Core 不感知传输层。
- 参数生成器会确定性生成 `build/generated_include` 转发头；删除生成目录后可从源码重新构建。
- Parameter Core 的运行期状态、事务及 get/set/reset 位于 `param.cpp`；持久化后端注册、save/load/status/erase 位于 `param_storage.cpp`，两者仅通过私有 `param_internal.hpp` 共享状态。公开兼容接口仍统一由 `param.h` 提供。
- TinyBSON 和 flashparams 使用调用者提供的固定或启动期 Buffer；编码/解码热路径不动态分配，不包含 fd、POSIX 或文件系统路径。
- Autosave 在首次变化后至少等待 300 ms，连续新保存间隔至少 2 s；异步存储小步按 10 ms 推进。ENOSPC 只进入可恢复暂停态，SD 从 unavailable 转为 available 时恢复一次受控保存，其他人工停用和 shutdown 不会被热插卡误唤醒。
- ParameterService 不持有或读取 Console，只负责 Core、FlashFS/FileStorage、Autosave 和 Flash 事务；在线参数由 MavlinkService 通过 Classic/Ext 协议访问。
- MavlinkService 在每次 LIST 开始前一次性激活全部 QGC 固定参数及 16 项公开 `SERIALx_BAUD/FUNCTION`，并在参数锁内冻结 used 句柄快照；整轮 PARAM_VALUE、按 index 缺帧补读以及快照内参数的 READ/SET 回包都复用该 count/index。固定参数仅允许同值写回；串口 Function 只允许 Disabled/RC Input，把目标端口设为 RC Input 时会在同一参数事务中禁用其他 RC owner，并向 QGC 回传全部受影响参数。
- SD/FatFs 在唯一固件配置中强制编译，不存在板级开关。FlashFS 是持续可用的主存储，SD 是带 generation 的镜像和恢复源；同 generation 还比较 payload CRC，差异时按既有 Flash 优先规则独立重建 SD。同一 TinyBSON 快照先分步写 Flash，再以临时文件分块写入、回读和 rename 轮换写入 SD 主/备文件。无 card-detect GPIO 时，ParameterService 每 3 秒通过 `disk_status` 软件探测；无卡启动只做一次初始探测，重新插入后执行 HAL deinit/init、重挂载并恢复 ENOSPC 或独立镜像重试。
- FlashFS 位于 `0x081E0000～0x08200000` 单个 128 KiB 扇区，采用追加记录和最终 commit 字；STM32H7 `FlashPartition` 负责 ECC 安全读、32-byte program、sector erase、回读和 cache 一致性。只有当前快照格式、CRC、BSON 结构、参数名和类型全部有效时才参与 generation 选择。
- 扫描不执行整段 cache invalidate；program/erase 成功后仅由 STM32 cache helper 失效实际修改范围，D-cache 关闭时 helper 防御性 no-op。
- DBECC 只有在活动安全读窗口、地址属于参数分区且 Bank 2 标志匹配时才允许 BusFault 恢复；其他 BusFault 一律进入启动诊断和复位。
- FlashFS 每条记录保存原始 payload 长度，CRC 只覆盖有效 payload；扫描选择同 token 的最后一条 CRC/commit 均有效记录。空间不足返回 ENOSPC，不自动擦除仍含有效快照的扇区。
- 不提供旧 `ParameterJournal`、旧键名或旧板级参数目录迁移。快照中出现未知参数名、类型不符或非当前格式时整份拒绝；开发板升级后直接擦除并按当前参数重新配置。
- 当前快照解码边界忽略 11 项只读 QGC 固定合同以及固定 Disabled 的 `RC_MAP_FLTMODE` 存储值；16 项 `SERIALx_BAUD/FUNCTION` 与其他可配置参数一样恢复并由 Autosave 持久化。当前固件未暴露恢复出厂命令，已删除无人驱动且无法完成异步票据的 erase API；部署清除仍由显式维护流程负责。
- 每次 load 都重新读取并验证条目 CRC；最新记录损坏或未 commit 时回退到更早的有效 FlashFS 记录。

## Application Runtime 生命周期

- `Param<T>` 构造只执行编译期类型约束，不调用 `param_get()` 或 `param_set_used()`；模块每次 start 必须显式 `bind()`，bind 失败时不得继续沿用上一 Runtime 的值。
- `param_shutdown()` 停止 Autosave，注销 notify/storage/lock callback，清除 ready、used、unsaved、动态 Layer、值 cache 和运行期同步对象；下一次 init 从未绑定状态开始。
- `ParameterService::shutdown()` 在释放自身 Mutex 前关闭 Parameter Core。FlashFS 无需显式关闭，FileStorage 的 SD 卡挂载和存储互斥量在进程生命周期内保持。
- Armed/Flash coordinator 仍独立于 Application Runtime，确保 ARMED 时 save/erase/confirm 延后；维护票据由 BootHealth 在严格 Disarmed 且 neutral/hard-safe 输出下批准，`appMain` 完成真实 IWDG reload 后激活。整个维护事务持有 arming interlock，按单调进度和硬截止时间推进；存储层没有 IWDG capability，也不得自行续命。
- 以上是源码生命周期契约。同上电 `shutdown → init → start`、掉电恢复、损坏回退和 ENOSPC 仍需目标板验收，不能由源码或主机生成结果代替。

## 当前验证边界

参数生成链当前确定性产出 203 项固件参数和 203 项公开 Metadata；JSON/XZ 与 CRC 由正式生成链统一给出。整机 `.text/.data/.bss`、签名镜像和 Factory HEX 属于全项目构建结果，只在 `docs/DIMA_SOURCE_MANIFEST.md` 记录最新 Windows 原生 clean build，避免本模块 README 复制一份易陈旧的资源数字。未新增或运行测试框架、测试文件、SITL 或仿真；MAVLink 在线调参、SD 物理拔插、掉电恢复、CRC 损坏回退和 ENOSPC 仍为实板验收项。
