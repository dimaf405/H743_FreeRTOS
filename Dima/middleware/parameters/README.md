# Dima Parameter 中间件

## 唯一参数定义与生成链

- Parameter YAML 工具固定为上游 commit `1f6b6f61f8f42eaab0269c16a442cb580f954d7c`；`tools/upstream/parameter_yaml_20260827/SOURCE_MANIFEST.json` 对原始脚本、schema、模板和 helper 做逐文件 SHA-256 闭包校验。
- `definitions/module_*.yaml` 是产品参数的唯一受版本控制定义。串口参数直接位于 `definitions/module_serial.yaml`，DroneCAN 参数直接位于 `definitions/module_dronecan.yaml`；二者与其他模块地位相同，不从 JSON/schema 或构建目录片段生成。
- 源码树禁止 `PARAM_DEFINE_*`、本地 C 注释 parser/renderer、`qgc_required` 扩展、手写参数目录和运行时参数名名单。

```text
Dima module_*.yaml
→ Tools/validate_yaml.py + validation/module_schema.yaml
→ Tools/module_config/generate_params.py
→ build/generated/parameters/module_params.c
→ src/lib/parameters/px_process_params.py
→ 初次 parameters.xml + parameters.json
→ 从官方 JSON 的单枚举或 min=max 自动生成 readonly_params.yaml
→ px_process_params.py --readonly-config（最终 XML/JSON）
→ px_generate_params.py --readonly-config
→ 上游原始暂存头
→ dima_parameters.hpp / dima::params
→ Dima 运行时合同与 Component Metadata
```

`module_params.c` 和上游原始头只是在构建目录中串接工具的中间产物，不进入源码树，也不得人工修改。原始脚本固定输出的文件名和 `px4` 命名空间仅保留在该暂存头；公开安装头机械适配为 `dima_parameters.hpp`、`dima::parameter_catalog` 和 `dima::params`，不改变枚举、表内容、类型或顺序。namespace 只读数组使用 C++17 `inline constexpr` 单一定义，避免不同翻译单元各保留一份；仍可用于编译期类型/容量检查，不增加运行期初始化。采用较新的 YAML 工具只替换生成方式，不导入上游主线的新参数、新默认值或新产品策略。

`readonly_params.yaml` 同样是构建目录中的自动派生物，使用上游支持的 block 模式；不是另一份受版本控制的参数定义或手写过滤名单。两遍生成之间逐项核对，只允许固定项增加 `readOnly: true`，禁止参数数量、顺序、类型、默认值或范围发生变化。

## 生成物与下游边界

- 参数数量、handle、类型、默认值、范围、枚举、单位、readOnly、volatile 与 reboot 语义完全由官方 XML、JSON 和生成头决定，不设置固定容量或第二份排序表。
- `parameter_contract.hpp`、公开转发头、Component Metadata JSON/XZ/Flash 数组、Dima 只读策略与持久化适配只读取官方产物，不重新解释 YAML 或中间 C。
- Component Metadata 使用主机侧 XZ preset 9 + extreme 压缩；JSON 内容、XZ/LZMA2 与 CRC64 格式保持。参数压缩文件 CRC、General 引用/公告 CRC 和嵌入数组由同一生成器同步更新；不手改压缩字节或保留旧 CRC。
- MAVLink Classic/Ext 参数协议按官方连续 handle 遍历完整目录；LIST、按 index 补读、READ/SET 与 ACK 使用同一目录索引，不再先维护一份 QGC/public 参数名单。
- `CAL_MAG1_ID`、`CAL_MAG1_ROT`、`CAL_MAG2_ID`、`CAL_MAG2_ROT`、`SENS_DPRES_OFF` 保持删除，不出现在 YAML、生成头、Metadata、参数协议目录、别名或虚拟参数中。当前产品在缺少这些参数时仍能正常进入并执行校准，这是本次重构必须保持的行为基线。
- 现有 `CAL_ACC0_*`、`CAL_GYRO0_*`、`CAL_MAG0_*` 等实际校准参数的名称、默认值、持久化和算法语义保持不变。

## 参数精简与展示分类

CAN 磁力计已删除 `MAG1_CAN_NODE`；来源节点号由首个合法磁场广播自动确定，运行时绑定不写回参数，设备 ID 继续使用生成合同，校准仍按设备标识匹配。

第一批精简删除 `GPS_1_PROTOCOL` 和 `RO_CAL_DIST`：前者的 Auto/6 原本使用同一 NMEA/UM982 驱动，后者改为内部 12 m 期望尺度并继续受固定圆、停车余量和超时约束。`COM_LOW_BAT_ACT`、`NAV_DLL_ACT`、`COM_RC_IN_MODE`、`MAV_SYS_ID`、`SYS_AUTOCONFIG` 被官方 QGC 5.1.3 直接读取，保留固定合同，不因缺少可配置动作而删除，也不放开其原取值范围。

- `Standard`：常用配置，权威 YAML 省略 category，由上游工具生成默认类别。
- `Developer`：高级 EKF、控制策略和诊断配置。
- `System`：校准/整定值及运行估计值；固定或单选合同集中到 `Compatibility` group。
- `RO_CAL_*` 集中到 `Rover Auto Calibration` group。RC 校准仍是 `Radio Calibration`，可变映射仍是 `RC Mapping`，六个模式槽仍属于 `Commander`，保持现有生成器的结构识别。

锁定的上游 YAML schema 只允许显式 `Developer/System`，不能因 QGC JSON 支持任意字符串就加入 `Advanced/Calibration` 并绕过校验。类别是标准 Metadata 展示字段，不改变权限、持久化、参数名或数值；QGC 仅特意将 Standard 置首，不会按类别名称自动隐藏、自动中文化或禁止修改。完整目录仍以生成 JSON 为准，决策和验收见 `docs/PARAMETER_SIMPLIFICATION_ZH.md`。

单枚举或 `min=max` 的参数现由生成器统一标记为标准 `readOnly`；默认值必须等于唯一合法值，矛盾时生成失败。单 bitmask 仍有关闭/开启两种状态，不按单枚举处理。当前 300 项中有 10 项只读，QGC 5.1.3 参数页勾选 **Hide read-only（隐藏只读参数）** 后列表为 290 项，普通浏览与搜索都过滤只读项。该 QGC 开关默认关闭，固件不能替地面站设置它，也不宣称重开页面后仍保持。

完整协议目录保留只读 Fact，确保 Radio、Airframe、安全状态等页面仍能读取产品合同。运行时固定约束和持久化加载过滤也使用同一单值识别规则，因此没有 min/max、只有一个枚举值的 `EKF2_HGT_REF` 同样拒绝非 GPS 值；不把只读展示误当作固件端写入校验。

## Parameter Core 与持久化

- 固件通过生成的 `dima::parameter_catalog::parameters`、`parameters_type` 与 `dima::Param<T>` 访问参数；`Param<T>` 构造不访问 Core，模块每次 start 显式 `bind()`。
- `Param<T, ID>` 保留生成枚举和编译期类型检查，运行期 bind/update 在 `param.cpp` 按 float/INT32/bool 共享实体。bind 成功后才标记 used 并提交缓存；update 不标记 used，未绑定时不读取 Core，失败清零并撤销绑定。bool 仍读取完整 INT32 后转换，不改变计数、原子候选、通知和重启生效语义，也不维护第二份参数表。
- Parameter Core 的运行期状态、事务及 get/set/reset 位于 `param.cpp`；持久化后端注册、save/load/status 位于 `param_storage.cpp`，公开兼容接口统一由 `param.h` 提供。
- TinyBSON 和 flashparams 使用调用者提供的固定或启动期 Buffer；编码/解码热路径不动态分配，不包含 fd、POSIX 或文件系统路径。
- ParameterService 与 Autosave 固定运行于独立低优先级 `wq:storage`；Autosave 在首次变化后至少等待 300 ms，连续保存间隔至少 2 s，并按 10 ms 小步推进。ENOSPC 进入可恢复暂停态，SD 从 unavailable 转为 available 后恢复受控保存。
- FlashFS 是持续可用的主存储，SD 是带 generation 的镜像和恢复源；同 generation 还比较 payload CRC，差异时按既有 Flash 优先规则重建 SD。
- 无 card-detect GPIO 时每 3 s 低频探测一次；重新挂载后等待 500 ms 再开始首个镜像写事务。介质级错误立即撤销 FileStorage/FatFs 的可用状态，下一次写入必须先完成重新初始化和挂载；失败不改变已经提交的 Flash 主副本。
- FlashFS 位于 `0x081E0000～0x08200000` 的单个 128 KiB 扇区，使用追加记录、最终 commit 字、32-byte program、回读与 cache 一致性。空间不足返回 ENOSPC，不自动擦除仍含有效快照的扇区。
- CRC/格式有效的旧快照若含当前目录已不存在的退役名称，只跳过对应条目；已知参数类型不符或快照格式无效时仍整份拒绝。当前固件不提供旧键别名或参数目录迁移表。

## Application Runtime 生命周期

- `param_shutdown()` 停止 Autosave，注销 notify/storage/lock callback，并清除 ready、used、unsaved、动态 Layer、值 cache 和运行期同步对象；下一次 init 从未绑定状态开始。
- `ParameterService::shutdown()` 在释放自身 Mutex 前关闭 Parameter Core。FlashFS 无需显式关闭，FileStorage 的 SD 挂载和存储互斥量在进程生命周期内保持。
- Armed/Flash coordinator 独立于 Application Runtime。运行期维护必须在 Disarmed 且输出 neutral/hard-safe 时由 BootHealth 批准，并持有 arming interlock；存储层没有 IWDG capability，也不得自行续命。

## 验证边界

正式 Make 入口会验证上游来源清单、YAML schema、XML/JSON/Header 一致性、派生 Metadata、五个删除参数的全闭包缺失和架构边界。未新增或修改测试文件、测试框架、runner、fixture、mock 或 test-only API。

Windows 生成、编译和 ELF 检查不能代替实板证明。同上电 `shutdown → init → start`、掉电恢复、损坏回退、ENOSPC、在线 MAVLink 调参、SD 空闲/写入中反复拔插时 HEARTBEAT 与遥测速率保持非零，以及 QGC 5.1.3 陀螺仪/加速度计/磁力计校准回归仍按最终报告标记 `BOARD/QGC PENDING`。
