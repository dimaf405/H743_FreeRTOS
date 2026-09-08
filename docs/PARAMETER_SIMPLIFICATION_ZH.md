# 第一批参数精简与分类记录

## 1. 结果与范围（2026-09-08）

第一批 7 项候选已完成审查和处理：删除 2 项，另外 5 项因官方 QGC 的直接依赖保留。当前权威生成目录从 **303 项变为 301 项**。不裁剪 18 路 RC、6 路 PWM、7 个串口、现有校准项目或安全能力。

本轮只在权威 YAML 中删除定义、设置标准 category/group；同步收口生产消费者并调用现有生成工具。没有新参数、私有 MAVLink wire 消息、虚拟兼容别名、手写派生目录或第二份参数注册表。本文件只记录决策和统计，完整逐项目录以 `build/generated/parameters/parameters.json` 为准。

## 2. 七项候选的实际处理

QGC 对照为官方 5.1.3 commit `7fe5b11b18a4c2eec17beb1b2a3ef45ac0c4e32e`。以下 QGC 路径相对该锁定源码；表中省略目录的 PX4 页面/控制器位于 `src/AutoPilotPlugins/PX4/`。没有修改地面站。

| 参数 | 处理 | 依据、影响与重要性 |
|---|---|---|
| `GPS_1_PROTOCOL` | 删除 | 固件 0/6 原来都使用同一 NMEA/UM982 实现；仅参与无效选择检查、日志和配置签名。QGC 运行 C++/QML/VehicleConfig 未发现硬依赖，通用参数页不再显示该项。低 |
| `RO_CAL_DIST` | 删除 | 内部保持原默认 12 m 期望尺度，按固定圆空间缩短；QGC 无专用消费者。旧自定义长度不再生效，不增加替代参数。低，但需验证实际运动/停车边界 |
| `COM_LOW_BAT_ACT` | 保留固定兼容项 | `src/FirmwarePlugin/PX4/PX4BatteryIndicator.qml:16`、`SafetyComponentSummary.qml:19`、`VehicleConfig/Safety.VehicleConfig.json:49` 直接取 Fact；删除会产生缺参告警/动作显示缺值。中。保留不代表已经实现低电量保护 |
| `NAV_DLL_ACT` | 保留固定兼容项 | `src/FirmwarePlugin/PX4/PX4MainStatusIndicator.qml:28`、`SafetyComponentSummary.qml:21`、`VehicleConfig/Safety.VehicleConfig.json:129` 直接读取。删除影响状态/安全显示并报警。中 |
| `COM_RC_IN_MODE` | 保留 RC-only 合同 | `PX4RadioComponent.cc:28/33/48`、`PX4AutoPilotPlugin.cc:176/187` 直接依赖 Radio 完成状态及 Flight Modes 前置。高，不删除 RC 消费者和安全检查 |
| `MAV_SYS_ID` | 保留固定身份 Fact | `AirframeComponentSummary.qml:16/26` 直接读取并显示 System ID。删除会空显示和缺参告警。中，不改变现有固定 1/1 编码/过滤身份 |
| `SYS_AUTOCONFIG` | 保留 | `AirframeComponentController.cc:28/29` 将它与 SYS_AUTOSTART 一起作为构造前置；缺少时提前结束初始化。参数页恢复默认及机架切换也有写入路径，但固件仍保持原固定 0、拒绝未实现动作。高 |

不能把缺少参数一概写成“QGC 必然崩溃”：`ParameterManager::getParameter()` 缺项返回默认 Fact，`FactPanelController::getParameterFact()` 可返回 nullptr，通常表现为缺参告警/空模型；SYS_AUTOCONFIG 则有明确的构造提前返回影响。保留 SYS_AUTOSTART 后也不能推导 Sensors 必然不可用。

## 3. 分类后的使用方式

锁定上游 `validation/module_schema.yaml:192` 只允许显式 `Developer`、`System`；未指定 category 时生成 `Standard`。虽然 QGC JSON 允许任意 category 字符串，本轮不修改锁定上游 schema，不绕过校验引入 `Advanced/Calibration` 自定义类别。

| 实际 category / group | 数量 | 用途 |
|---|---:|---|
| `Standard` | 70 | 常用配置：RC/模式映射、PWM 功能/方向、串口、DroneCAN、安装位置、运行范围、校准半径/停车/兜底速度等 |
| `Developer` | 73 | 高级调优：EKF 噪声/延迟/健康检查、导航与手动控制策略、校准激励上限、传感器采样/诊断及高级日志设置 |
| `System` 中非 Compatibility 分组 | 148 | RC 校准、传感器校正、PWM 端点、前馈/增益/整形、RTK/水平结果及运行估计值；其中 EKF2_MAG_DECL 仍是 volatile RAM 值，不是持久化标定 |
| `System / Compatibility` | 10 | 固定或单选产品/地面站合同；包括本批保留的五项，另有 SYS_AUTOSTART、RC_MAP_ROLL/PITCH、NAV_RCL_ACT 和 EKF2_HGT_REF |
| **合计** | **301** | 三个顶层 category；System 总数为 158 |

分类不改变参数读写权限，也不意味着所有 System 项都能自动校准。例如 PWM 端点仍需人工确定，完整磁软铁矩阵仍不可由当前平面运动自动完成。

`RO_CAL_*` 集中到 `Rover Auto Calibration` group，其中半径、停车上界、兜底速度在 Standard，纵向/转向激励 ceiling 在 Developer。原有 `Radio Calibration`、可变 `RC Mapping`、六槽 `Commander` 和 Logger/Sensor 分组合同保留；固定 RC 完成标记移到 Compatibility，不进入可变 RC 映射生成集合。

QGC `FactMetaData.cc:1233/1446` 读取标准 category/group；`ParameterEditorController.cc:184` 动态建树、`:230` 将 Standard 置首。它不会仅凭 Developer/System 名称自动隐藏、自动中文化或禁止修改。本轮没有新增“高级解锁”开关或确认按钮。

## 4. 运行与旧配置边界

- GPS 端口仍由 `SERIALx_FUNCTION=GPS` 选择；GPS/SBUS 各自唯一 owner、不得共端口、UM982 生成波特率、普通 8N1 与 SBUS 专属 100000 8E2/换回流程不变。重配置仍经 Disarmed 与 maintenance 批准。
- 直线为 `min(12 m, R_work-distance-0.5 m)`，不足 5 m 拒绝；RAM 路径继续受 12 m 内部尺度、固定圆和杆臂余量约束。不增加安全半径，不改变 180 s 运动周期/600 s 会话上限，也不改变每轮输出、速度、slew、TTL 或 watchdog 门禁。
- 目录、handle、Metadata 和日志参数计数统一再生成；旧存储仍按参数名而不是旧 handle 读取。容量内、CRC/格式及已知参数类型合法的旧生产快照可跳过两个退役项，保留其他配置；不新增旧键别名或迁移表。
- 旧 `.params` 文本导入可能提示这两个名称不存在，需移除对应两行再导入；不承诺地面站静默忽略。
- 升级后应重新连接并完整获取参数和新版 Component Metadata，确认三个 category 与 Compatibility/Auto Calibration 分组。UI 缓存刷新、重连和真实旧快照加载仍需实板/QGC 验收。

### 旧生产快照容量审查

现有 BSON 公式是 `5 + Σ(type byte + UTF-8 name + NUL + value bytes)`；Float 使用 8 B wire 值，Int32 使用 4 B。

| 项目 | Payload 上界 |
|---|---:|
| 旧 303 项全目录 | 6127 B |
| 本次退役两项 | 41 B |
| 新 301 项生成容量 | 6086 B |
| 旧正常生产快照保守上界 | 5939 B |

生产保存只编码非默认用户层，排除 volatile；9 项生成固定项的 165 B 和 EKF2_MAG_DECL 的 23 B 不进入正常用户保存。因此旧生产上界 `6127-165-23=5939`，仍比新容量小 147 B，且已包含本次两项可能保存的非默认值。本轮不增加存储 RAM 或改快照格式。

人工构造包含所有默认/固定/volatile 的完整 303 项二进制文件可能先被容量检查拒绝；它不是当前生产保存链可产生的快照，不在上述兼容承诺内。

## 5. 非测试静态验收

执行环境：Windows 原生 `E:\freertos\H743_FreeRTOS`，项目缓存 Arm GCC 10.3.1。没有新增/修改测试、框架、runner、fixture、mock、Host Test、SITL 或仿真；没有调用 upload、刷机、板卡串口或车辆动作。

```powershell
Set-Location E:\freertos\H743_FreeRTOS
& C:\Users\master\.local\bin\make.cmd NO_COLOR=1 `
  parameter-generated parameter-metadata-verify
& C:\Users\master\.local\bin\make.cmd -j4 NO_COLOR=1 `
  dima_rover uorb-generated-verify mavlink-generated-verify `
  parameter-metadata-verify logger-generated-verify
git -c core.safecrlf=false diff --check
```

已完成的证据：

- 官方参数生成和 Component Metadata 校验通过：301 项；XML/JSON/生成头、连续参数协议和 Logger 计数一致。
- 与本轮开始的 303 项生成 JSON 按名称逐项比较：只少 GPS_1_PROTOCOL/RO_CAL_DIST，无新增；其余 301 项除 category/group 外的全部属性一致，214 项展示分类或分组变化。类型、默认值、范围、单位、枚举、volatile/reboot 属性没有漂移。
- 架构 `PASS - 454 first-party source files`，43 个 uORB schema、49 个 Logger Topic、8 种 Profile 组合；MAVLink 仍为锁定 230 条官方 wire 定义，无私有消息变化。
- 发布构建在共享 HEAD/固件身份更新后触发 `unplanned action MavlinkIdentity.cpp`。每次保留 build、在该身份生成后用同一命令重跑，先完成 `[49/49]`，再对最终 `a6d046d` 身份完成 `[16/16]`，均 exit 0；不是 clean build，没有绕过架构或进度门禁。
- Application/MCUboot 未解析符号为空，向量 `0x08040400`，ELF、签名/Factory 布局及 watchdog prepare/feed 链通过。

最新资源为 Application `text/data/bss=625096/12688/571136` bytes；Flash `637824/782336`（81.5%）、SRAM `590752/884736`（66.8%）、D2 `182912/262144`（69.8%）。这是当前工作树总占用，不是本功能独立性能或实板峰值证明。

### 制品记录

制品对应 Git 身份 `a6d046d2cc8c96a0b0314c9f41c462eb627d6263`；image digest：`0aaabc2c00a006bafa5303d425bd3fa5c06ee9a542f4c768e22e357f64f8c05d`。

| 制品 | 大小（bytes） | SHA-256 |
|---|---:|---|
| `build/H743_FreeRTOS.elf` | 11128856 | `f1caec369a3f368e2536c68075325fb7ef80feee49621b7dc154c885f7e40998` |
| `build/H743_FreeRTOS.bin` | 637824 | `ce9db20107df1c89f7f240df746aacb4f17e326c6e349c121cfc366ba174ce4e` |
| `build/H743_FreeRTOS_signed.bin` | 638998 | `4f3d3e3efad6e3c3fa465d2799b9ae7ce192ee0aafc95e35a5cb87b67c5e2129` |
| `build/H743_FreeRTOS_factory.hex` | 1654044 | `2d3d8ae0c5f6b845605141aba51a83de94a352d4b475a448b6504997d693a849` |
| `build/mcuboot/mcuboot.elf` | 2684396 | `14e52c94c4e68c91e679a748f0ad8ad9e03d341134175722a653d6447565f54d` |
| `build/mcuboot/mcuboot.bin` | 48308 | `84b49887296b34822cefff28e9fc8b6b1d7b9251e1d44c3640f1ad4e8c073dcc` |

签名容器哈希仅标识这一份 ECDSA 签名样本，不要求重签字节相同。tracked `git diff --check` 为 0；新增本文的 `git diff --no-index --check` 无空白诊断（exit 1 仅表示新文件存在差异），变更中测试/框架/runner/fixture/mock/Host Test/SITL/仿真路径为 0。

## 6. 工作区与待实板项目

本轮开始 HEAD 为 `3ac342f808e847fe5b9c223ccb9457fec33f29e8`；期间其他工作流提交使 HEAD 先移至 `6dd7182e2084cc79519ba75eacb8272158a9f109`，并在 `a6d046d2cc8c96a0b0314c9f41c462eb627d6263` 身份完成本次制品验证。之后至 `7d19695` 的共享提交只修改 README/文档，不改变本次已验证的生产代码或参数定义。部分已有/本轮改动被纳入这些共享提交；本代理没有执行 commit、push、reset 或 clean，不能把当前 git diff 当作本轮完整增量。

观察到其他会话的 upload/build，未操作其进程或串口；正式构建在没有观察到并发 make 时启动。并发文档更新按最新内容保留，不整体回退日志、存储、RC 或其他工作。

仍待单独授权：QGC 参数目录和分类缓存更新、Radio/Flight Modes/Airframe/安全状态页无新增缺参告警、旧生产快照及 `.params` 导入、真实串口重配置、固定圆内校准路径/停车和掉电回滚。本轮结论只到源码、权威生成、架构和 Windows 制品层面。

## 7. 单值参数的只读与隐藏（2026-09-08 后续变更）

为使只有一个合法值的参数不占用可调列表，生成器从上游首轮 JSON 自动识别单枚举或 `min=max`，生成构建目录内的 `readonly_params.yaml`，再将它传给上游 `px_process_params.py` 与 `px_generate_params.py` 的正式 `--readonly-config` 入口。最终 XML、JSON、头文件和 Component Metadata 使用同一只读集合，不新增手写参数名列表、不修改上游脚本或生成物。默认值与唯一合法值冲突时停止生成；只有一个 bit 的 bitmask 仍可开关，不当成单值。

当前协议总数仍为 **301 项**，其中 **10 项标记 readOnly**，过滤后的可调列表为 **291 项**。类型、默认值、范围、枚举、分类和 handle 顺序保留；既有固定值约束从 9 项扩为 10 项，使原来仅有一个枚举值的 `EKF2_HGT_REF` 也进入 MAVLink 写入校验和旧快照加载过滤，保持 GNSS-only 高度合同。第 5 节的制品记录是此前参数分类变更的证据，不代表本节新增只读行为。

对照的 QGC 仍为官方 5.1.3 commit `7fe5b11b18a4c2eec17beb1b2a3ef45ac0c4e32e`：

- `FactMetaData.cc` 读取标准 `readOnly`；`ParameterEditorController::_buildListsForComponent()`、`_factAdded()`、`_shouldShow()` 在启用隐藏开关时过滤只读项，涵盖分类列表和搜索。
- 参数页已有 **Hide read-only（隐藏只读参数）** 复选框；`ParameterEditorController.h` 中 `_hideReadOnly` 默认是 `false`。升级固件并获取新版 Metadata 后，需要勾选该项；固件不能替用户改变 QGC 默认设置，也不承诺重新打开页面后继续保持。未勾选时这些参数仍显示为只读。
- Radio/Airframe/安全状态页面仍从完整参数目录获取固定 Fact。彻底从 MAVLink 目录删除这些项会影响这些页面，本实现通过 QGC 自身的列表过滤保留依赖。

本节已完成的非测试静态验收：

- Windows 原生执行第 5 节的 `parameter-generated parameter-metadata-verify` 及完整 `dima_rover uorb-generated-verify mavlink-generated-verify parameter-metadata-verify logger-generated-verify` 命令，均最终 exit 0。
- 与改动前 301 项 JSON 逐属性比较，只增加 10 个 `readOnly: true`；上游头除只读数组外逐字一致。XML、JSON、Component Metadata、上游只读数组和运行时固定约束集合一致，10 项只读、291 项可调，没有新增或删除参数。
- 已在最终 Application BIN 中找到新版完整参数 Metadata XZ 字节，确认生成数据进入固件；ELF 的固定约束数组为 120 B，对应 10 项。
- 首轮构建的进度终检报告 `41/100 actions`、59 个 unclaimed；保留构建目录和日志、确认未观察到仍运行的 make 且 HEAD 不变后，原命令重跑 `[7/7]` 完成。未绕过进度或架构门禁，未修改构建进度工具；该计数异常没有在重跑时复现，不将其根因归为已证实的并发构建。
- 架构 `PASS - 454 first-party source files`，43 个 uORB schema、49 个 Logger Topic、301 个参数、8 种 Profile 组合；MAVLink 仍是锁定的 230 条官方 wire 定义。Application/MCUboot 无未解析符号，向量 `0x08040400`，签名、Factory 布局和 watchdog prepare/feed 链通过。

本节制品基于 HEAD `121b8885357d377b5365413067d1300635f73b3b` 加本地未提交变更，image digest 为 `2d87e14dd377c854e8ff0d03fa8730e367130d049118e9d7c1cebf11e4d5437e`。Application `text/data/bss=625136/12688/571136` bytes；Flash `637864/782336`、SRAM `590752/884736` bytes。

| 制品 | 大小（bytes） | SHA-256 |
|---|---:|---|
| `build/H743_FreeRTOS.elf` | 11129176 | `8e13cd1c883268a933e06f7cb7ba069c7dc2ab8680bac6dbcbab374982592d4e` |
| `build/H743_FreeRTOS.bin` | 637864 | `ded2a702dcd8d897d004903c1180949e99b40c23657c34884130e0779c3b1b2c` |
| `build/H743_FreeRTOS_signed.bin` | 639039 | `3b6658d79e66ef1b369af70a6b3629424c52e2c72b5c72294c8a061c55d8e383` |
| `build/H743_FreeRTOS_factory.hex` | 1654139 | `f1f24ab4e7c90a47a1cd7b6c6129b2c90955b3c3e57c1f72e2c079d91abe7bcd` |

没有新增或修改测试文件、测试框架或测试基础设施，也没有手工编辑参数/消息派生列表。未刷机、操作 QGC 或占用板端串口；QGC Metadata 刷新、隐藏开关及相关页面交互仍为 `BOARD/QGC PENDING`。
