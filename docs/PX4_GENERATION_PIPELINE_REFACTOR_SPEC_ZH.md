# PX4 原生消息与参数生成链重构规格

## 1. 目标与行为基线

Parameter、uORB 和 MAVLink 三条标准生成链由锁定版本的 PX4/MAVLink 上游原始工具产生。Dima 只保留权威产品输入、构建编排、Windows 入口适配、原子安装和消费标准产物所必需的薄运行时合同，不维护“功能等价”的 parser、字段布局器、renderer、codec、ID/CRC 表或消息/参数目录副本。

本次重构只替换生成方式，不主动改变 Parameter Core、FlashFS/SD、uORB Runtime、MAVLink handler、传感器校准算法和产品参数策略。当前固件在缺少以下五个参数时仍可正常校准，这是用户确认并必须保持的重构前行为基线：

- `CAL_MAG1_ID`
- `CAL_MAG1_ROT`
- `CAL_MAG2_ID`
- `CAL_MAG2_ROT`
- `SENS_DPRES_OFF`

上述名称不得进入 YAML、生成头、Metadata、MAVLink 参数目录、兼容别名或虚拟参数。现有 `CAL_ACC0_*`、`CAL_GYRO0_*`、`CAL_MAG0_*` 等实际校准参数保持名称、默认值、持久化和算法语义不变。不修改 QGroundControl 5.1.3 源码或安装目录。

## 2. 固定上游来源

| 生成域 | 固定来源 | commit/version | 本地来源闭包 |
|---|---|---|---|
| uORB | PX4-Autopilot v1.17.0 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | `tools/upstream/uorb_v1_17/SOURCE_MANIFEST.json` |
| Parameter YAML | PX4-Autopilot main 快照 | `1f6b6f61f8f42eaab0269c16a442cb580f954d7c` | `tools/upstream/parameter_yaml_20260827/SOURCE_MANIFEST.json` |
| MAVLink definitions | mavlink/mavlink | `33af200d25ec6f0925b49b1ba82bbf1294ea5f72` | `tools/mavlink/mavlink.lock.json` 中的 XML SHA-256 |
| MAVLink generator | pymavlink 2.4.47 | `fcaa2c7d25e3169dc66155929c338487941555e9` | archive SHA-256 与完整源码树聚合 SHA-256 |

上游脚本、helper、schema、模板和参考消息保留原版权头与相对路径。`tools/generation/source_manifest.py` 动态扫描 vendored 目录并验证逐文件 SHA-256；文件增删、内容漂移或 commit 标识变化都会使正式生成或架构门禁失败。Windows/入口适配只存在于 Dima 薄编排层，不修改上游原件。

## 3. Parameter：唯一 PX4 YAML 链

### 3.1 权威输入

- `Dima/middleware/parameters/definitions/module_*.yaml` 是产品参数唯一受版本控制的定义格式。
- 现有参数由 PX4 原始 `migrate_c_params.py` 机械迁移，并核对名称、类型、默认值、范围、枚举、单位、volatile 与 reboot 语义；源码树不再保留 `PARAM_DEFINE_*` 入口。
- 串口参数直接写入标准 PX4 `module_serial.yaml`，参数描述保持 `SERIALn`、物理 UART/USART n 与 TX/RX 引脚一一对应；官方参数生成器和包装器均不包含串口特化，也不派生串口专用头。STM32 句柄、DMA、IRQ 与 GPIO token 留在板级实现。DroneCAN schema 仍只能在 `build/generated` 生成 PX4 YAML 片段，再进入相同正式链。
- 采用新 YAML 工具不代表引入 PX4 主线的新参数或默认值；当前产品参数集合和策略不变。

### 3.2 正式链

```text
PX4 YAML
→ Tools/validate_yaml.py + validation/module_schema.yaml
→ Tools/module_config/generate_params.py
→ build/generated/parameters/module_params.c
→ src/lib/parameters/px_process_params.py
→ parameters.xml + parameters.json
→ src/lib/parameters/px_generate_params.py
→ px4_parameters.hpp
→ Dima 薄合同与 Component Metadata
```

`module_params.c` 只是在构建目录中串接两段官方工具的中间产物，不入库且不得修改。`parameter_contract.hpp`、公开转发头、只读策略、Flash 表和 Component Metadata 只能读取官方 XML、JSON 或生成头，不重新解释 YAML 或中间 C。

### 3.3 目录与协议

- 参数数量和 handle 由官方生成数组长度推导，不维护固定总数。
- MAVLink Classic/Ext 协议按连续官方 handle 遍历完整目录；LIST、按 index 补读、READ/SET 与 ACK 使用同一目录索引。
- `qgc_required`、`kQgcRequiredParameters`、`kMavlinkPublicParameters` 等第二份名单均已退役。QGC 页面兼容不能通过本地字段、虚拟参数或别名绕开唯一 YAML 源。
- 五个删除名称在任一权威输入或派生产物中出现即触发门禁失败。

## 4. uORB：PX4 v1.17 原始生成器

### 4.1 schema

- `Dima/messages/schemas/*.msg` 是唯一消息输入，文件名使用 PascalCase。
- schema 只使用 PX4 原生语法，包括 `ORB_QUEUE_LENGTH` 和 `# TOPICS`；禁止 `@queue`、`@alias`、`@external`、`@abi` 等本地扩展。
- 对 vendored PX4 快照中存在唯一同名文件的产品 schema，架构门禁逐字节比较完整字段、常量、单位、Topic alias 和 `MESSAGE_VERSION`，不得按消费者裁剪。
- Dima 专用消息也由同一生成器处理，不能在 C++ 中复制结构布局。

### 4.2 生成与 Runtime 适配

`tools/uorb/generate_messages.py` 直接执行未修改的 `px_generate_uorb_topic_files.py`、helper 和 EmPy 模板，生成：

- `uORB/topics/*.h` 与每消息 `.cpp/.json`
- `uORBTopics.hpp/.cpp`
- Topic ID、消息 hash、队列长度和 alias
- `uorb_sources.mk`
- 从官方 `ORB_DECLARE` 动态派生的旧 `.hpp` include-only 转发头

Runtime 直接消费官方 `orb_get_topics()` 与 `orb_topics_count()`。本地 `uORB.h/uORB.hpp` 只提供编译和运行所需的薄接口。ABI lock、手写 Topic registry、producer/consumer 名单、`.dima_orb_meta`、`MetadataRegistrar`、linker 边界符号和对应 ELF 断言均已退役。

`build/generated/uORB/.generated.json` 记录 schema、上游生成脚本/模板和全部派生产物的 SHA-256；架构门禁动态核对输入/输出集合、官方头/源/JSON 三元组、ID/hash、aggregate registry 与 Make fragment，不写入消息名称或固定数量。

## 5. MAVLink：wire 定义与运行策略分离

### 5.1 wire 唯一来源

- `tools/mavlink/message_definitions/dima.xml` 是唯一 wire 根，只 include 固定版本的 `common.xml`。
- 标准消息全部由固定 definitions 继承；产品消息只能在 `dima.xml` 中定义。Python/C++ 不复制消息 ID、字段、CRC、payload、codec 或枚举目录。
- Make 直接执行锁定 pymavlink 的 `mavgen.py --lang C --wire-protocol 2.0 --no-validate`。host-tools 不固定 lxml，因此显式关闭可选 XSD；固定 XML hash、ElementTree well-formed/include 检查、mavgen 解析、生成符号和完整输出 hash 闭包共同提供跨主机一致的失败边界。
- 当前 `dima.xml + common.xml` 生成 230-message wire 闭包。wire 中存在消息不等于固件宣称实现其业务能力。

### 5.2 产品运行策略

`Dima/modules/mavlink/mavlink_runtime.yaml` 只声明实际发送频率、请求行为和 inbound handler。`tools/mavlink/generate_runtime_contract.py` 仅从 mavgen 已生成头验证消息符号并生成 C++ 调度合同：

- 所有 ID 使用 `MAVLINK_MSG_ID_*` 宏，不写数值 ID；
- YAML 不参与 wire 编解码；
- handler、发送频率和产品能力不能改变 CRC、payload 或方言；
- `.generated.json` 闭合 XML、lock、bootstrap、策略、薄生成器和全部 mavgen 输出。

旧裁剪方言/codec 生成器及在 lock 中复制的消息/枚举/策略列表已删除。MAVLink 上游头以 system include 编译，只隔离其自身 packed-member 告警；项目第一方源码仍保持 `-Werror`。

## 6. 校准协议不变量

重构后必须保留当前实际校准链：

- 接受 `MAV_CMD_PREFLIGHT_CALIBRATION` 中 gyro `param1=1`、mag `param2=1`、accel `param5=1`、Radio `param4=1` 及既有取消语义；
- 保留 PX4 风格 `COMMAND_ACK` 结果与 source system/component 定向回复；
- 长事务继续用 `[cal] calibration started/progress/orientation/side done/done/failed/cancelled` STATUSTEXT 驱动 QGC；
- 实际结果仍写入现有 `CAL_*0` 参数并由原持久化链保存；
- 缺少五个已删除参数时不增加恢复、别名或特殊分支。

本规格不修改 QGC。用户已确认当前缺少五参数仍能校准；本轮代码生成和 Windows 构建只验证该行为没有被静态接口破坏。只有明确执行 QGC 5.1.3 实板回归后才记录对应结果，未执行时不增加占位状态。

## 7. 构建集成与门禁

- Make 只声明 `.msg`、Parameter YAML/板级 schema、MAVLink XML 和运行策略 YAML 为权威输入，派生产物统一写入 `build/generated*`。
- host-tools 固定 EmPy、pyros-genmsg、PyYAML、Cerberus、Jinja2 和 pymavlink 来源；正式构建不依赖个人 site-packages 或联网分支。
- 关键 wrapper、输入选择、原子安装、兼容接口、公式和控制流使用中文说明；上游原文件保留原注释与版权头。
- 架构门禁拒绝源码参数 C 定义、本地参数/uORB parser/renderer、MAVLink codec/ID/CRC 表、手写 registry、五个删除参数和来源/派生 hash 漂移。
- 不新增或修改任何测试文件、测试框架、runner、harness、fixture、mock 或 test-only API。

## 8. Windows 正式验收

所有正式 Git、生成、编译和验收在 Windows PowerShell 的 `E:\freertos\H743_FreeRTOS` 中执行；WSL 只负责发起 Windows 进程。

```powershell
make NO_COLOR=1 clean
make -j4 NO_COLOR=1 parameter-generated
make -j4 NO_COLOR=1 uorb-generated
make -j4 NO_COLOR=1 mavlink-generated
make -j4 NO_COLOR=1 uorb-generated-verify
make -j4 NO_COLOR=1 mavlink-generated-verify
make -j4 NO_COLOR=1 dima_rover
make -j4 NO_COLOR=1 verify
make -j4 NO_COLOR=1 parameter-metadata-verify
make -j4 NO_COLOR=1 check-architecture
git diff --check
```

还必须核对 Application/MCUboot ELF layout、`nm -u`、连续两次生成内容 hash、五个删除名称的全闭包缺失、PX4 同名 schema 与固定快照一致，以及 `git diff -- tests` 为空。

最终验收报告只记录实际执行并通过的源码、静态、Windows 生成和构建检查。在获得实板操作授权前不上传、不刷写、不修改 QGC，也不把 Windows 构建结果描述为 QGC 5.1.3 校准证明；未执行的实板工作不追加占位状态。
