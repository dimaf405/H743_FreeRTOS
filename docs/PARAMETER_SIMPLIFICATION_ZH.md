# 第一批参数精简与分类记录

## 1. 结果与范围（2026-09-08）

第一批 7 项候选已完成审查和处理：删除 2 项，另外 5 项因官方 QGC 的直接依赖保留。当前权威生成目录从 **303 项变为 301 项**。不裁剪 18 路 RC、6 路 PWM、7 个串口、现有校准项目或安全能力。

本轮只在权威 YAML 中删除定义、设置标准 category/group；同步收口生产消费者并调用现有生成工具。没有新参数、私有 MAVLink wire 消息、虚拟兼容别名、手写派生目录或第二份参数注册表。本文件只记录决策和统计，完整逐项目录以 `build/generated/parameters/parameters.json` 为准。

## 2. 七项候选的实际处理

QGC 对照为官方 5.1.3 commit `7fe5b11b18a4c2eec17beb1b2a3ef45ac0c4e32e`。以下 QGC 路径相对该锁定源码；没有修改地面站。

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
- 首次发布构建因共享 HEAD/固件身份生成收敛触发 `unplanned action MavlinkIdentity.cpp`。保留 build、同命令重跑后 `[49/49]` exit 0；不是 clean build，没有绕过架构或进度门禁。
- Application/MCUboot 未解析符号为空，向量 `0x08040400`，ELF、签名/Factory 布局及 watchdog prepare/feed 链通过。

最新资源为 Application `text/data/bss=625096/12688/571136` bytes；Flash `637824/782336`（81.5%）、SRAM `590752/884736`（66.8%）、D2 `182912/262144`（69.8%）。这是当前工作树总占用，不是本功能独立性能或实板峰值证明。

### 制品记录

image digest：`39ce21db5fcc31248148033f5c688291982c0e3ffeb2bbffdefc201d95255d49`。最终文件哈希在交付检查后填入。

## 6. 工作区与待实板项目

本轮开始 HEAD 为 `3ac342f808e847fe5b9c223ccb9457fec33f29e8`；期间其他工作流提交使 HEAD 移至 `6dd7182e2084cc79519ba75eacb8272158a9f109`，部分已有/本轮改动也被纳入该共享提交。本代理没有执行 commit、push、reset 或 clean，不能把当前 git diff 当作本轮完整增量。

观察到其他会话的 upload/build，未操作其进程或串口；正式构建在没有观察到并发 make 时启动。并发文档更新按最新内容保留，不整体回退日志、存储、RC 或其他工作。

仍待单独授权：QGC 参数目录和分类缓存更新、Radio/Flight Modes/Airframe/安全状态页无新增缺参告警、旧生产快照及 `.params` 导入、真实串口重配置、固定圆内校准路径/停车和掉电回滚。本轮结论只到源码、权威生成、架构和 Windows 制品层面。
