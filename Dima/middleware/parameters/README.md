# Dima Parameter 中间件

## 唯一参数定义与生成链

- Parameter YAML 工具固定为 PX4 commit `1f6b6f61f8f42eaab0269c16a442cb580f954d7c`；`tools/upstream/parameter_yaml_20260827/SOURCE_MANIFEST.json` 对原始脚本、schema、模板和 helper 做逐文件 SHA-256 闭包校验。
- `definitions/module_*.yaml` 是产品参数的唯一受版本控制定义。串口参数直接定义在 `definitions/module_serial.yaml`；DroneCAN 参数仍由其 schema/manifest 在 `build/generated` 生成 PX4 YAML 片段，两者统一并入同一条正式链。
- 源码树禁止 `PARAM_DEFINE_*`、本地 C 注释 parser/renderer、`qgc_required` 扩展、手写参数目录和运行时参数名名单。

```text
PX4 Parameter YAML
→ Tools/validate_yaml.py + validation/module_schema.yaml
→ Tools/module_config/generate_params.py
→ build/generated/parameters/module_params.c
→ src/lib/parameters/px_process_params.py
→ parameters.xml + parameters.json
→ src/lib/parameters/px_generate_params.py
→ px4_parameters.hpp
→ Dima 薄运行时合同与 Component Metadata
```

`module_params.c` 只是在构建目录中串接两段 PX4 官方工具的中间产物，不进入源码树，也不得人工修改。采用较新的 YAML 工具只替换生成方式，不导入 PX4 主线的新参数、新默认值或新产品策略。

## 生成物与下游边界

- 参数数量、handle、类型、默认值、范围、枚举、单位、volatile 与 reboot 语义完全由官方 XML、JSON 和生成头决定，不设置固定容量或第二份排序表。
- `parameter_contract.hpp`、公开转发头、Component Metadata JSON/XZ/Flash 数组、Dima 只读策略与持久化适配只读取官方产物，不重新解释 YAML 或中间 C。
- MAVLink Classic/Ext 参数协议按官方连续 handle 遍历完整目录；LIST、按 index 补读、READ/SET 与 ACK 使用同一目录索引，不再先维护一份 QGC/public 参数名单。
- `CAL_MAG1_ID`、`CAL_MAG1_ROT`、`CAL_MAG2_ID`、`CAL_MAG2_ROT`、`SENS_DPRES_OFF` 保持删除，不出现在 YAML、生成头、Metadata、参数协议目录、别名或虚拟参数中。当前产品在缺少这些参数时仍能正常进入并执行校准，这是本次重构必须保持的行为基线。
- 现有 `CAL_ACC0_*`、`CAL_GYRO0_*`、`CAL_MAG0_*` 等实际校准参数的名称、默认值、持久化和算法语义保持不变。

## Parameter Core 与持久化

- 固件通过官方 `px4::parameters`、`parameters_type` 与 `px4::Param<T>` 访问参数；`Param<T>` 构造不访问 Core，模块每次 start 显式 `bind()`。
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

正式 Make 入口会验证上游来源清单、PX4 YAML schema、官方 XML/JSON/Header 一致性、派生 Metadata、五个删除参数的全闭包缺失和架构边界。未新增或修改测试文件、测试框架、runner、fixture、mock 或 test-only API。

Windows 生成、编译和 ELF 检查不能代替实板证明。同上电 `shutdown → init → start`、掉电恢复、损坏回退、ENOSPC、在线 MAVLink 调参、SD 空闲/写入中反复拔插时 HEARTBEAT 与遥测速率保持非零，以及 QGC 5.1.3 陀螺仪/加速度计/磁力计校准回归仍按最终报告标记 `BOARD/QGC PENDING`。
