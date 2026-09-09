# 代码体积与逻辑收敛记录

当前第四轮结果见第 8 节；前文保留各轮归档证据，不覆盖历史基线。

## 1. 范围与完成状态

2026-09-08 按“保持现有功能与安全合同”执行第一轮整顿：先建立 Windows 制品基线，再处理有实际链接证据的重复和已退役接口，最后用现有生成、架构及发布门禁复核。

| 步骤 | 本轮完成内容 |
|---|---|
| 基线 | 归档 ELF/BIN/map、参数 XML/JSON/原始头及改动前源码，使用同一 Arm GCC 10.3.1 release 配置 |
| 格式化减重 | SD ULog 的两处字符串/整数格式化复用已有 Format，不再拉入第二套 newlib formatter |
| 只读表去重 | 公开生成参数表使用 C++17 inline constexpr，由链接器合并重复 TU 表 |
| 逻辑收敛 | 删除无人调用的旧数组事务入口、响应辅助 API 与无用跟踪，保留实际算法和安全路径 |
| 核验 | 权威输入/输出一致、目标构建/ELF/签名通过，记录具体符号和共享工作树统计边界 |

没有裁剪自动校准阶段、官方 QGC 兼容、18 路 RC、6 路 PWM、串口、存储恢复或日志 Profile；没有调整安全阈值、缓冲/任务栈容量、EKF 浮点选项、newlib 版本或编译优化等级。没有新增测试文件、框架、runner、fixture、mock、Host Test、SITL 或仿真，没有刷机或车辆动作。

## 2. 实际修改及等价边界

### 2.1 ULog 共用已有格式化器

`SdLogWriter.cpp` 原来两处 `std::snprintf` 分别格式化参数 key 的 `"%s %s"` 和硬件 UID 的 `"%016llX"`。链接 map 证明它们引入 `snprintf → _svfprintf_r → _dtoa_r`，而 `dima::format::format_to/npf_vsnprintf` 本来已被固件使用。

替换只涉及上述字符串/整数格式：返回完整所需长度、截断拒绝、左补零、大写十六进制、16 字节 UID 正文及 NUL 保持；ULog 顺序、背压、路由和过滤不变。没有改浮点日志、stdout/setvbuf 或 `nano.specs`。未链接的 printf/状态打印函数不计作本次 Flash 节省。

### 2.2 生成参数表单一定义

原始 PX4 头使用 namespace-scope `static constexpr` 数组，多个消费者可能分别保留表。公开安装头在已有命名空间适配之外，仅机械改为 C++17 `inline constexpr`；原始头、参数枚举、表内容、类型和顺序不重渲染。

`ConstLayer` 返回值拷贝，参数核心按 handle/字符串内容访问，持久化保存名称/类型/标量值；没有生产消费者依赖某个 TU 私有表的地址身份。内部 C++ 表的链接性和地址身份确实变化，但 C 接口、参数值、MAVLink 和持久化格式不变；constexpr 能力和常量初始化保留，不增加初始化顺序或运行时 guard。

R331 仍逐字核对经过上述限定变换后的整个公开头，上游 source manifest/原工具链不变。原始 header/XML/JSON 的 SHA 与基线一致。ELF 的 `V` 表示 weak object，并不是 volatile。

### 2.3 校准接口与统计收敛

- 删除没有生产调用的 `CalibrationParameters::replace_provisional()`。当前唯一关联更新路径仍是命名参数 `revise_float/apply_revisions`；实际使用的磁 `refine`、最终保存及回滚全部保留。
- 删除未接入生产的 `ResponseStatistics::minimum/noise_stddev` 和匹配比值辅助 API。
- 只删除通用平台统计中无用的最小值跟踪；`TransientSlope` 的最小值和幅值/激励门禁仍保留。
- `gain_lower_bound(direction, noise)` 固定使用全部现有消费者所需的 PreShaping 坐标；真实 ARX、SLEW 与完整电机曲线仍使用相应的 applied 数据，不改变其物理辨识或拒绝条件。

这四个校准文件净减 91 行；连同格式化及生成/一致性适配，本轮清理自身在 7 个代码文件中净减 79 行，不包含并发目录迁移及生成链改造。`CalibrationParameters.*` 已由其他会话移到 `Dima/rover/modes/auto_calibration/`，清理后的内容完整保留，没有恢复旧路径。无消费者函数原本可能已经被链接 GC 丢弃，不能按删除行数推算 Flash 收益。没有为“更短”合并具有不同稳定窗口、ready 条件和回滚语义的 poll 状态机，也没有合并协调器/RoverDifferential 的独立安全检查。

## 3. 前后制品与可归因证据

| 指标 | 修改前基线 | 第一轮末次通过工作树 | 差值 |
|---|---:|---:|---:|
| Application BIN / Flash load span | 637864 B | 620440 B | −17424 B，约 17.0 KiB / 2.73% |
| text（含只读数据） | 625136 B | 608076 B | −17060 B |
| data | 12688 B | 12324 B | −364 B |
| bss | 571136 B | 571136 B | 0 |
| 总 SRAM 地址占用 | 590752 B | 590400 B | −352 B |
| D2 data 地址占用 | 182912 B | 182560 B | −352 B |

Flash 占槽比例由约 81.5% 降到 79.3%。地址跨度包含对齐空隙，因此 SRAM/D2 差值不必与 data 差值相同；这些不是运行时栈/heap 高水位。

共享工作树期间有其他会话的 readonly Metadata、IMU FIFO、遥测、模式目录整理及跨平台构建工作。readonly 工作已纳入归档基线；后续改动也进入重构后的镜像。因此上表是同工具链前后**整棵共享工作树的差值，不是无并发变量的单补丁净收益**。未回退或冒领这些并发修改，目录移动也不计作删除功能或本轮删行收益。

本轮修改可以直接确认的主要符号：

| 符号/数据 | 基线 | 重构后 |
|---|---:|---:|
| `parameter_catalog::parameters` | 2 × 2408 B | 1 × 2408 B |
| `parameter_catalog::parameters_type` | 2 × 301 B | 1 × 301 B |
| `snprintf` | 136 B | 不再链接 |
| `_svfprintf_r` | 6476 B | 不再链接 |
| `_dtoa_r` | 3288 B | 不再链接 |
| `npf_vsnprintf` | 已存在 44 B | 仍为 44 B |
| `setvbuf` | 已存在 432 B | 仍为 432 B |

参数两表明确消除 2709 B 重复只读数据；格式化引入链及其部分转换/locale 辅助项消失。不能把库对象在 map 开头出现就认定它最终占 Flash，必须结合最终已定义符号和分配区核对。

## 4. 生成与发布验证

正式目录为 `E:\freertos\H743_FreeRTOS`，编译器为项目缓存 Arm GCC 10.3.1。编译配置原来就是 release `-Os`、function/data sections 和链接 GC，本轮未使用 LTO 或改变这些选项；保留规范 ELF 的调试和校验信息。

```powershell
Set-Location E:\freertos\H743_FreeRTOS
& C:\Users\master\.local\bin\make.cmd NO_COLOR=1 firmware-identity-generated
& C:\Users\master\.local\bin\make.cmd NO_COLOR=1 parameter-generated parameter-metadata-verify
& C:\Users\master\.local\bin\make.cmd -j4 NO_COLOR=1 `
  dima_rover uorb-generated-verify mavlink-generated-verify `
  parameter-metadata-verify logger-generated-verify
git -c core.safecrlf=false diff --check
```

基线首次构建在 SIZE 步失败，同期确认另一原生 make 正在写共享 build；没有终止他人的进程或删除 build。进程结束后用同一命令确认基线 `[7/7]` exit 0 并归档；修改后发布目标首次 `[312/312]` exit 0（BIN 620448 B）。随后一次并发构建只完成 `288/376`，没有把其编译/签名成功当作完整门禁通过。单独检查并确认无其他原生 make 后，使用 Windows 原生 PowerShell 7 重跑上列完整目标，末次 `[152/152]` exit 0（BIN 620440 B）。这些是完整发布目标的依赖重编译，不是本代理执行 clean 后的构建；没有修改进度门禁来掩盖并发失败。

当前主机选择由已有 `make/host.mk` 负责：本次 Windows 制品仍在 `build/`，公开参数安装头现为 `build/generated_include/parameters/dima_parameters.hpp`。已核对新的模式路径、生成器和 R331，未覆盖其他会话的跨平台及目录整理改动。

- 架构通过 454 个首方源文件；Application/MCUboot 未解析符号为空，向量 `0x08040400`，签名/Factory 布局和 watchdog prepare/feed 链通过。
- 301 个参数、43 个 uORB schema、49 个 Logger Topic、8 种 Profile 组合保持；MAVLink 仍是锁定的 230 条官方定义。
- 参数 JSON SHA：`b1bbe20b73e0b48976a3021cc9ba65323a1ad2124ec09f31fa83886c49201020`。
- 参数 XML SHA：`5f69f49a55ac4421e6e090595fa6306b8bf43a666b6d9c6ede28ab2c5bb4bd20`。
- 上游原始参数头 SHA：`e843ae89fb7c38ec608d30b944c4258c890d5edeff32c35a342a752db6c1323b`。

上述三份生成物与基线逐字节一致；本轮没有改变参数定义、类别、默认值、只读/volatile 属性或消息 payload。

基线 image digest 为 `2d87e14dd377c854e8ff0d03fa8730e367130d049118e9d7c1cebf11e4d5437e`，BIN SHA 为 `ded2a702dcd8d897d004903c1180949e99b40c23657c34884130e0779c3b1b2c`。只读基线副本位于本机临时目录 `C:\Users\master\AppData\Local\Temp\dima-code-size-c9ac5919ad0f4a5287f4e907192c591f`，保留用于核对，没有清理用户原始制品。

末次制品于 2026-09-08 16:38（UTC+8）复核，工作分支为 `feature/dima-phase3`，HEAD 为 `121b8885357d377b5365413067d1300635f73b3b`，工作区仍含未提交修改。image digest 为 `c66ad1c6bd2323a853beabd3a1b2f397de34e0aa0b7911539c63d27ee0270735`。

| 制品 | 字节数 | SHA-256 |
|---|---:|---|
| `build/H743_FreeRTOS.elf` | 10997980 | `444bd29485b6fad3dabba8bb80fa43b07946468d6ef7a281d6483b5da102062e` |
| `build/H743_FreeRTOS.bin` | 620440 | `0dd26e443cc01e82cbfb71c5cb5eb836548a7a9544e2021355eb470927949bc7` |
| `build/H743_FreeRTOS_signed.bin` | 621615 | `fbb166964f7bdb0c4bc8e0a9468942a2e27b1b7345ddbb438ff7c586c49791c8` |
| `build/H743_FreeRTOS_factory.hex` | 1612219 | `e120a7b858667199e7d6d0c1f7976be962e045b2894a7fffcdeff87d1854b724` |
| `build/mcuboot/mcuboot.bin` | 48308 | `84b49887296b34822cefff28e9fc8b6b1d7b9251e1d44c3640f1ad4e8c073dcc` |

该轮验收 ELF 再次确认参数两表各一份，`snprintf/_svfprintf_r/_dtoa_r` 不再链接。`git -c core.safecrlf=false diff --check` 通过；没有新增/修改测试或仿真路径，实现验收阶段未提交或推送。实板/QGC/车辆验收仍未执行，不由静态构建代替。

## 5. 后续整顿顺序与防止再次膨胀

1. **先查真实链接成本。** 新增功能说明 text/data/bss 与主要引入链；优先用现有 Format、生成表、控制器和事务，不为小调用引入第二套库。不按 ELF 调试文件大小或源码行数判断 Flash。
2. **清理被正式替代的入口。** 保留一个明确的生产调用路径；移除经过调用闭包确认的旧接口，不保留“也许以后用”的并行实现。不同稳定窗口、错误处理和安全所有权不是可随意合并的重复。
3. **继续审查生成只读数据的表达。** Logger/Profile 等表可以检查是否存在可共享数据，但必须保持全部 Profile、输出和时间语义；未测量前不承诺收益，也不新增手写 Topic/参数目录。
4. **LTO 单独评估。** 如果仍需要更大幅度减重，用隔离的相同源码版本做 release A/B，检查弱符号、初始化、C ABI 和 EKF 浮点合同，再单独验收；本轮没有直接开启。
5. **RAM 最后按实测收敛。** heap、任务栈、日志 Ring、DMA 和传感器缓冲必须有目标板峰值、溢出/恢复及调度证据，不能凭静态“空余”缩减。

本轮没有新增自动化规则冻结模块、符号、精确体积或消息清单；继续复用已有通用门禁。若进一步减重需要裁剪真实校准、导航、日志或外设能力，先明确产品取舍，再另行实施。

## 6. 第二轮：全项目扫描与 Logger 合同收敛

按最终 ELF 的只读对象、库引入链和生产调用检查全项目；上一轮参数表已各保留一份。EKF 的单/双精度数学表以及不同后端的同名静态对象不是重复实例，不能按符号短名直接合并。本轮优先处理有明确冗余的 Logger，不改控制算法、安全快照或缓冲容量。

实施顺序及验收点：

1. 归档本轮通过基线，保持上一轮 620440 B BIN；不以旧源码路径或并发目录移动计算收益。
2. 在已有 Logger 生成器中共享相同采样策略，去掉运行期未读取的 Topic 名称、ID 和回调标志副本。全部 49 个 Topic × 8 个 Profile 的 kind/interval 必须保持一致，名称/ID 仍由 uORB metadata 提供，回放回调仍由生成的独立索引表提供。
3. 合并 producer 的重复拷贝选择和固定频率/源频率发送结果处理。固定频率仍取最新数据，源频率仍有界逐 generation 排空，停止边沿仍取最新状态；保留 dropout、游标、背压重试与错误处理。
4. 用现有 Logger/参数/uORB/MAVLink 生成校验、架构和 Windows `dima_rover` 门禁复核，比较最终符号及制品。不要为此次精简新增测试、runner、私有协议或手写表。

基线在 Windows 原生完整目标 `[8/8]` exit 0 后归档至 `C:\Users\master\AppData\Local\Temp\dima-code-size-round2-bb3c1e50cbe4477f9dc821e37e2b9c1f`，包含制品、生成语义清单及改前源码。

### 6.1 已实施与等价核对

- `generate_logger_contract.py` 从原有 `merged_sampling()` 的完整 kind/interval 结果生成共享只读表；每个 Topic 的八种组合仍分别生成索引。索引超过 uint8 容量会拒绝生成，不截断或环绕；没有压缩间隔单位或在运行期重新计算策略。
- 删除无人读取的 TopicPolicy ID/name/replay_callback 字段，保留 disposition 与 flush_on_stop。uORB metadata 和独立的回放注册索引仍为实际消费者提供对应数据，Topic ID 的生成静态断言全部保留。
- `SdLogWriter` 仅在 `source_rate && !newest_only` 时调用逐 generation 拷贝；其他情况共用 latest 拷贝。普通发送把固定频率视为 burst=1，与有界 source-rate 共用结果处理；固定频率时间门禁、Blocked 游标回退、NoData 停止排空及 Failed 关闭路径不变。
- 前后生成清单的全部 Topic 语义和输入哈希逐项一致，覆盖 49 × 8 = 392 组 kind/interval；ULog wire、sidecar、参数 JSON/XML/上游原始头逐字节一致。仅改变本地只读表表示，没有新增参数、消息或 QGC 协议。
- 本轮只有两个代码文件发生新增修改：生成器 22 行新增/11 行删除，producer 7 行新增/19 行删除。净行数几乎不变；实际收益来自共享数据和减少分支，源码行数与 Flash 占用分开记录。

### 6.2 实际制品与链接成本

| 指标 | 第二轮基线 | 第二轮后 | 差值 |
|---|---:|---:|---:|
| Application BIN / Flash load span | 620440 B | 617088 B | −3352 B，约 3.27 KiB / 0.54% |
| text | 608076 B | 604724 B | −3352 B |
| data / bss | 12324 / 571136 B | 12324 / 571136 B | 不变 |
| SRAM / D2 data 地址占用 | 590400 / 182560 B | 590400 / 182560 B | 不变 |
| Topic + Sampling 只读表 | 3920 B | 490 + 64 = 554 B | −3366 B，约 85.9% |
| `SdLogWriter::append_topic` | 632 B | 612 B | −20 B |
| `SdLogWriter::drain_topics` | 284 B | 252 B | −32 B |
| 生成 `sampling_policy` 查表函数 | 44 B | 52 B | +8 B |

共享策略多一次常量索引读取，但没有动态分配、运行期合并或无界扫描。不能把表和函数差值简单相加代替 BIN 的实际跨度；对齐及其他链接选择仍影响最终布局。规范 ELF 含调试信息，本轮 ELF 文件反而略大，不影响 Flash 实际下降。

相对首轮最初 637864 B 基线，当前整树累计减少 20776 B（约 20.3 KiB / 3.26%），Flash 槽占用由约 81.5% 降到 78.9%。累计值继续保留第 3 节的并发归因边界。

### 6.3 验收与制品身份

首次重编译期间其他会话将 HEAD 从 `121b8885357d377b5365413067d1300635f73b3b` 推进到 `211b95b9e8b16124ad49709c2cf2d639bc58cd98`，固件身份更新触发 `unplanned action CXX ... MavlinkIdentity.cpp`。确认没有其他原生构建进程后原样重跑，第 4 节的完整 Windows 命令 `[17/17]` exit 0。没有执行 clean、修改进度门禁或代替其他会话提交；本轮表和函数的符号变化可直接核对，但跨 HEAD 整树比较仍包含固件身份更新。

架构通过 454 个首方源文件；参数、Logger、uORB、MAVLink 生成检查、Application/MCUboot ELF、签名及 Factory 布局全部通过，未解析符号为空。`git diff --check` 通过，测试/框架/仿真路径没有新增修改。该轮实现验收阶段未提交、推送、刷机或执行板端动作；新的调度峰值与 SD/QGC 运行行为仍需实板复核。

2026-09-08 16:55（UTC+8）的 image digest：`4857d26e533c3abd33a163feb94f87b3e557d6748e8f280e463329838499c3cd`。

| 制品 | 字节数 | SHA-256 |
|---|---:|---|
| `build/H743_FreeRTOS.elf` | 11001696 | `2127c06f829bd0bd97603f494911fff43704396e45a2ad18a28e78fa7bda0464` |
| `build/H743_FreeRTOS.bin` | 617088 | `953ff8f5f7391cd9a6c30e661000e94e4cd20df9a3ecd7cdf1b65d8bef5472a4` |
| `build/H743_FreeRTOS_signed.bin` | 618263 | `0252eb72ee25601ecc782898bd03d10c2e430034dc2b06724f46f3b6a26968b4` |
| `build/H743_FreeRTOS_factory.hex` | 1604150 | `97dd040c1a380b62673a8f39520d667733719a7f400f7e5cf027a2923681bc01` |

## 7. 第三轮：模块重复逻辑与退役接口

本轮按成员引用、内联接口、稀疏状态引用和重复代码块检查 `modules/rover/middleware`，再逐项追踪真实消费者、回调与生命周期。低引用次数只用来筛选，不能据此删除仍被单一消费者使用的发布器、订阅器或 EKF 状态。

实施与验收顺序：

1. uORB：移除没有消费者的 `MessageFormatReader::readUntilFormat/readNextField/bufferLength`；保持整个格式组的解压、展开、余留字节和错误状态处理，上游工具快照及生成格式不动。
2. Commander：共用逆注册顺序的回调注销与调度排空，保留注册顺序、逐项失败原因、Error/Stopped 区分、Disarm 和撤销会话的原有位置。
3. SensorCalibration：共用前端代次/校正值确认谓词，但按既有阶段保留正向/回滚差异；Gyro/Accel 回滚不新增原始样本门禁，Mag 正向应用仍要求校准计数变化或饱和后的新输出。两个等待状态的超时、终态和互锁所有权不合并。
4. 复用现有生成、架构、Windows 完整构建与制品门禁；不新增测试、框架、手写参数/消息列表或硬件动作。

保留项：差速层和执行器层各自的负向安全锁存/完整快照检查仍独立，执行器还有自己的 hard-safe 锁存；任务与 ISR 调度入口的上下文/唤醒条件不同；MAVLink 旧命令兼容入口仍有生产消费者。以上不作为废弃分支删除。

基线在其他会话完成模式目录提交后，以 HEAD `e1f839ec989b98419f9293ca223c2dec7021611e` 先生成固件身份，再完成 Windows `[16/16]` exit 0。BIN 为 617088 B，SHA-256 为 `cb2e00fd7a29f631eef0cf7d4969798c539744024dccdbbba70cf323e34c57e6`。改前源码与制品归档至 `C:\Users\master\AppData\Local\Temp\dima-code-size-round3-b2b58183d9044ce59fb4d0c6930f20a5`；本批六个代码文件在归档时均无未提交修改。

### 7.1 完成结果与复核边界

三项均已落地，六个代码文件合计新增 48 行、删除 195 行，净减少 147 行：uORB 净减 111 行，Commander 净减 11 行，SensorCalibration 净减 25 行。uORB 的三个旧接口已没有源码/工具调用残留；两个原有完整格式组函数保持原实现。

Commander 的公共清理被六个注册失败出口、stop 与 Error 共用；清理之外的 Disarm、会话撤销、失败提示和状态更新顺序保持。正常 stop 仍在调度排空前后各 Disarm 一次，后一次覆盖已在执行中的回调，不作为重复操作删除。

SensorCalibration 的公共谓词仍在原有阶段分发下调用：`rollback=false` 化简为原正向条件，`rollback=true` 化简为原回滚条件。Level 自动校准的并发改参检查仍位于正向等待入口，Mag 回滚不新增正向计数推进要求；失败、超时和互锁释放继续由两个等待处理函数分别负责。没有修改参数写入/保存流程、消息字段或 QGC 终态协议。

| 指标 | 第三轮基线 | 第三轮后 | 差值 |
|---|---:|---:|---:|
| Application BIN / Flash load span | 617088 B | 616784 B | −304 B |
| text | 604724 B | 604420 B | −304 B |
| data / bss | 12324 / 571136 B | 12324 / 571136 B | 不变 |
| SRAM / D2 data 地址占用 | 590400 / 182560 B | 590400 / 182560 B | 不变 |
| Commander 启动、停止、Error 及公共清理函数合计 | 764 B | 620 B | −144 B |
| 校准两条等待处理及公共前端确认函数合计 | 876 B | 720 B | −156 B |
| uORB `readMore + expandMessageFormat` | 452 + 140 B | 452 + 140 B | 不变 |

退役的 uORB 函数在基线中已被链接 GC 丢弃，删除它们主要减少维护代码，不宣称额外释放等量 Flash。函数体与最终 BIN 分别统计，不能忽略对齐后直接相加。相对首轮最初基线，当前整树累计减少 21080 B（约 20.6 KiB），Flash 槽占用约 78.8%；累计值仍保留前两轮的并发归因边界。

### 7.2 2026-09-09 最终验收

源码首次完整构建 `[29/29]` exit 0。补充 Disarm 时序中文说明后的跨日复核在 MAVLink 生成校验失败：差异仅列出四个方言的 `version.h` 和生成清单，实际头部差异为 `MAVLINK_BUILD_DATE` 从 `Tue Sep 08 2026` 变为 `Wed Sep 09 2026`，wire 版本仍是 `2.0`。项目运行源码没有使用该日期宏。

外层 Make 进度入口未传递 `--what-if` 的预期刷新效果，因此用现有内部生成配方明确刷新缓存，再回到完整外层入口验收：

```powershell
& C:\Users\master\.local\bin\make.cmd DIMA_BUILD_INTERNAL=1 `
  --what-if=tools/mavlink/mavlink.lock.json NO_COLOR=1 mavlink-generated
& C:\Users\master\.local\bin\make.cmd -j4 NO_COLOR=1 `
  dima_rover uorb-generated-verify mavlink-generated-verify `
  parameter-metadata-verify logger-generated-verify
```

完整目标 `[9/9]` exit 0，架构通过 454 个首方源文件，Application/MCUboot 未解析符号为空，签名及 Factory 布局通过。上述内部入口仅用于执行已有生成配方，没有绕过最终架构/生成/制品门禁，没有手改生成头、修改 wire 或忽略日期差异；上游生成器仍采用日历日期，跨日缓存以后仍可能需要同样刷新，本轮未把它改成固定日期。

Logger/uORB 生成清单以及参数 JSON/XML/上游原始头与本轮基线逐字节一致；没有新增/修改测试、框架或仿真路径，`git diff --check` 通过。上述制品验收时 HEAD 为 `e1f839ec...`，该轮实现验收阶段未提交、推送、刷机或访问板端串口；后续提交使用新的固件身份，不能直接沿用这里的制品哈希。静态/构建通过不替代 QGC、传感器与车辆验收。

最终 image digest：`0638d901255b6f019204d3149abb4912694983d3af74a847e770fdaab5734ceb`。

| 制品 | 字节数 | SHA-256 |
|---|---:|---|
| `build/H743_FreeRTOS.elf` | 10998736 | `d098bfd895107b6043ab96dcfdb61522cfb7d3fa4b7b770109046cad5226b71b` |
| `build/H743_FreeRTOS.bin` | 616784 | `869e14d7aa6d004cc132eaa3318420aec569df2ebdd1e0fd9c83dda8df5658d9` |
| `build/H743_FreeRTOS_signed.bin` | 617960 | `7803a5700834f00fb6c3015a747535f3718209291761b30a5da1b26e2d51ec0b` |
| `build/H743_FreeRTOS_factory.hex` | 1603427 | `1ae5341402bfff499d872c47329ca8b9acfbd19d246a54e20ea5d9b8f6c1c8be` |

## 8. 第四轮：MAVLink 只读应答与会话清理

本轮只修改 `MavlinkParameterExt.cpp`、`MavlinkParameters.hpp`、`MavlinkMetadataFtp.cpp/.hpp`：共享 EXT 编码发送入口，直接消费生成的类型/容量；完整 FTP reset 复用会话清理，超时复用完整 reset，协议 ResetSessions 直接清理会话并保留 ACK。任务进度观察与断链复位的状态保留语义不同，Flash/SD 写入分支的恢复职责也不同，本轮不合并它们。

边界核对：EXT 字符串格式、索引拒绝及未找到哨兵不变；空发送回调之前仍执行编码，保持序号推进；FTP 完整复位清缓存而协议关闭不提前清本次应答。`MAV_PARAM_EXT_TYPE_INT32/REAL32` 仍为 6/9，生成的名称/值容量仍为 16/128，未修改任何权威参数、消息、XML 或生成工具。

2026-09-09 Windows 基线 `[7/7]` exit 0，HEAD 为 `10650e7dd5a08bb864a0cdd8c7106be551d24d71`。BIN 616792 B、SHA-256 `27541dcb9d707d6bc2d22c8295d6fc295c095c92b3694e79858c42922388e2f7`；DTCM 60896 B、SRAM 530784 B、D2 data 172096 B。这些数值已包含其他会话的 DTCM/链接布局修改，不与上轮直接比较来计算本批收益。基线源码、生成清单与制品归档到 `C:\Users\master\AppData\Local\Temp\dima-code-size-round4-d99d241dde90492497e3d536dfba1758`；本批四个代码文件在归档时均无未提交修改。

完成后四个代码文件新增 24 行、删除 37 行，净减 13 行。Windows 完整目标 `[153/153]` exit 0，架构通过 454 个首方源文件；MAVLink/uORB/Logger 生成清单与参数 JSON/XML/上游原始头均与基线逐字节一致，签名/Factory 布局和未解析符号检查通过。没有新增或修改测试、框架及仿真路径，`git diff --check` 通过。

| 指标 | 第四轮基线 | 第四轮后 | 差值 |
|---|---:|---:|---:|
| Application BIN | 616792 B | 616752 B | −40 B |
| EXT 应答处理及编码函数合计 | 550 B | 526 B | −24 B |
| FTP reset / expire / reset_session 合计 | 116 B | 100 B | −16 B |
| DTCM / SRAM / D2 data 地址占用 | 60896 / 530784 / 172096 B | 60896 / 530784 / 172096 B | 不变 |

EXT 统计包含基线中 110 B 的 `mavlink_msg_param_ext_value_encode.constprop.0.isra.0`，它在新镜像中被内联进公共回复函数；不能只比较两个调用者就漏算原有编码体。本批收益属于小幅维护性收敛，不宣称大幅代码减重或 RAM 节省。未修改或提交并发的 DTCM、启动、组合根和 IMU/遥测代码，也未进行刷机。

该轮实现验收时的 image digest 为 `2fc124dc58c1b94e21fbb4e55ee719e8c600883073bf8827394173f8142a56f6`；BIN SHA-256 为 `bb87e483a3f8dd01301376398ff508a30d46ccf7a8c04085b0c0775eebc7427d`，signed BIN 617926 B、SHA-256 为 `d6c98cb3d7c096f323f8b32925d90fa09f1138b949501f5ebaa38eddd3962899`。前文三轮的四个提交不包含本批内容；本批后续提交使用新的固件身份，不能直接沿用这里的制品哈希。
