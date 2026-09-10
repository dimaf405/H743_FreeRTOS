# 构建到上传四批优化记录

## 当前构建策略：只做显式验证

自 2026-09-09 本轮调整起，日常命令不自动执行独立检查，以下历史记录中的强制门禁策略不再适用。

| 命令 | 当前行为 |
|---|---|
| `make dima_rover` / `make firmware` | 生成、编译、签名并生成 MCUboot/Factory 产物，不做独立校验 |
| `make dima_rover upload` / `make upload` | 仅准备 OTA 镜像并执行上传协议，不做主机独立校验 |
| `make upload-ready` | 准备所选 OTA 镜像和主机依赖，不访问串口、不校验镜像 |
| `make verify` | 显式运行架构、Metadata/Logger、ELF、签名及 Factory 完整校验 |
| `make upload-verify` | 显式检查本地上传 ELF 和签名；外部 `UPLOAD_IMAGE` 只检查该包签名 |
| `make app-check` / `make check-architecture` / 对应 `*-verify` | 显式执行相应单项检查 |

日常编译沿用上传的直接调度方式，跳过进度 dry-run 预演。编译前仍先运行权威生成工具，避免身份或参数等生成输出更新时间戳后漏编译消费者。镜像签名、生成器自身的输入合法性约束、工具缓存身份和板端 MCUboot 验证属于产物生成或运行机制，保持原流程；不手写参数或消息列表。上传日志以 `IMAGE_HASH` 表示读取 TEST 请求所需摘要，不再把读取摘要标为主机签名验证。

并行标志改在配方执行时求值，保留 GNU Make 此时才补齐的显式 `-jN` 和 jobserver，避免自动并行度覆盖用户参数。Linux 使用原生 Python/GCC 和独立 Linux 输出目录，Windows 使用其本机工具链及 `build/`。

### 本轮验证边界

在原生 WSL/Linux 下使用独立 `BUILD_DIR=build-linux-explicit-validation-20260909` 执行 `make -j4 NO_COLOR=1 dima_rover`，完整生成、Application/MCUboot 编译、签名与 Factory 合并通过；实际日志没有 `ARCH`、`META_VERIFY`、`LOG_VERIFY`、`ELF` 或 `VERIFY` 独立检查阶段。该目录从空产物开始，本次会话耗时 409.49 s；不能与前述 Windows 增量数据或不同源码/缓存状态比较。

日常组合 `make -n dima_rover firmware upload-ready upload` 的独立检查集合为空；显式 `make -n verify upload-verify upload` 包含全部五类检查。外部 `UPLOAD_IMAGE` 的 `upload-verify` 只展开指定包签名检查。显式检查通过前，上传目标不会开始访问设备。

共享 `build-linux/` 中检测到其他会话的编译上传，本轮基线已主动停止，未停止其他会话；因此不提供优化前后的百分比结论。未访问板端串口或执行刷机，没有新增或修改测试基础设施。

首次完整构建之后，共享 Mission 源码继续变化。后续 `upload-ready` 先因 `ApplicationContext` 调用三参数构造函数、`MissionService` 已改为两参数而失败；调用在其他会话中收敛后复跑，又因 `MissionStorage.cpp` 的 `load_initial_state/load_next_item` 等定义与头文件不匹配而失败。本轮没有改动这些源码，也没有把失败耗时记作增量性能数据。错误分别保存在 `/tmp/h743-upload-ready-after.log` 和 `/tmp/h743-upload-ready-retry.log`；当前最新共享源码不能据此前的构建结果宣称通过。

对首次成功构建留下的应用 ELF 显式运行 `tools/verify_application_elf.py` 已通过，向量为 `0x08040400`；对其 signed BIN 显式运行项目 `imgtool.py verify` 也通过。`upload-verify` 的本地/外部包入口已通过命令展开核对，但其后续实际 Make 运行未到达，因为前序上传准备被上述并发源码错误阻断。本轮修改文件的 Python AST 和 `git diff --check` 均通过。

## 原四批优化范围（历史记录）

保留 `make dima_rover upload`、Windows 原生执行、权威参数/消息生成及全部架构、ELF、签名和设备身份门禁。不新增测试文件、框架、runner 或仿真，不执行上传/刷机，不自动结束其他会话。

1. 计时、启动并行度和编译缓存：代码完成。保留显式 `-jN`，默认按可用内存和 CPU 选择 1～8 路；缓存只加速 Application 编译，不缓存链接/签名验收。`DIMA_BUILD_TRACE=1` 复用既有进度包装器记录逐对象耗时，正常快速路径不增加逐对象 Python 进程。
2. 生成依赖和失效范围：代码完成。按目标已有 include 分类二次展开生成前置条件，首次构建不依赖旧 `.d`；参数/uORB/Metadata 未变化内容保留 mtime，MAVLink 保持整树原子安装并继承未变文件时间戳；对象目录从真实对象闭包派生，每目录只执行一次 mkdir。仍保留 Makefile 变化的保守失效规则，通过 ccache 按真实命令复用对象，不引入另一套编译命令清单或遗漏编译选项变化。
3. 编译热点：代码完成。采样最慢的前两项为 ApplicationContext/app_main；组合根对四个引用成员改用前置声明，完整类型只在 `.cpp` 解析。`app_main.d` 中对 EKF 实现头的直接/传递依赖由 38 个降到 0。未改变对象布局、优化级别、浮点合同或功能，不增加 PCH/Unity Build。
4. 上传侧重复工作：主机代码完成。Windows 一轮枚举复用 present 集合；COM 已排他占用时不再同轮启动第二次协议打开。保持窗口 1、MTU 512、完整等待期限、物理绑定及身份闭环；窗口对比和端到端实板性能留待授权。

## 基线与并发边界

初始源码 HEAD 为 `fa9a900`。主目录存在其他会话的参数、RC 和安全改动，均不属于本批。

首个基线使用 `BUILD_DIR=build/speed-baseline-20260909`，50.14 s 后因生成目录在并发清理中消失而失败；不作为性能数据。第二个隔离工作树受 Windows checkout 换行转换影响，上游来源哈希校验拒绝，51.91 s 后失败；日志还显示重新安装 host-tools 时发生 PyPI 网络重试。保持来源校验，不通过更新哈希接受换行漂移，改用保持 Git 原始换行的隔离基线。

后续记录必须区分工具缓存为空、对象缓存为空、编译缓存命中、正常增量及上传各阶段；失败构建不能作为优化收益分母。签名、制品与实际板端性能分开报告。

## 已验证结果

有效隔离树为 `E:\freertos\h743_speed_base_lf_20260909`，固定 `fa9a900` 源码，仅叠加本轮变更，不包含共享工作区同时进行的 RC/参数及大规模头文件拆分。旧输出目录以 `build-before`、`build-trace`、`build-cached` 留存；日志位于 Windows Temp 的 `dima-speed-*.log`，会话 JSON 路径由命令末尾输出。

| 场景 | 墙钟时间 | 证据边界 |
|---|---:|---|
| 原规则从空对象目录构建 | 124.92 s | 包含 host-tools 重装、PyPI 网络等待，不能当成纯编译时间 |
| 优化后空对象目录、关闭 ccache | 76.38 s | `-j4 DIMA_CCACHE=off upload-ready`，主机工具已准备 |
| 优化后空对象目录、缓存已填充 | 52.79 s | `-j4 upload-ready`，296/296 编译调用命中 |
| 本轮中间版本，无改动复跑 | 21.76 s | 仍受 Make 空格条件错误影响，重复架构扫描 |
| 修正后无改动复跑 | 7.86 s | 无编译、无完整架构扫描，保留 Metadata/Logger 校验 |

以上是同一台低可用内存主机上的单次观测，不承诺固定耗时，也不将不同工具缓存状态的差值全部归为编译器收益。`-j2 DIMA_BUILD_TRACE=1` 的一次验证为 84.37 s、294/296 命中，只用于定位热点，不与 `-j4` 数值直接比较。

隔离源码优化前后 BIN SHA-256 均为 `213b35df3310bb8b89153dab5695493e6a5a39c86fc8c3a7ed045186dba18deb`，大小 602196 B，text/data/bss 为 590872/11284/571104 B；所有非隐藏生成文件逐字节一致。其来源输入、版本与共享工作区后续源码不同，不作为后续组合固件的身份或资源证明。

## 修正的主机构建问题

- 原架构强制目标列表的续行缩进留下空格，使 Make `if` 把空列表当成真，即使内容指纹 current 也重复全扫描。已在该列表生成处 `strip`；显式完整目标仍保持强制扫描。
- Logger 和 MAVLink 的成功标记必须与未变化的代码文件区分：前者成功后刷新 mtime，后者保留 mtime；否则生成输入改过、输出内容相同时会在每次 Make 中重复生成。最终复核已包含该修正。
- 原主机缓存根和镜像版本使用递归 shell 变量，反复启动 Python；现在每个 Make 进程只求值一次，缓存根还固定导出给递归 Make。
- Python 工具环境现在按 ABI/依赖/安装配方内容寻址，位于与旧目录并列的 `host-python-envs`。原型曾放在旧 `host-python` 内，被并发旧安装器整体替换；该失败已保留日志并修正。新安装器有跨进程 OS 锁，失败不发布成功标记，输出路径限制为专用散列缓存目录。
- ccache 只在缓存二进制通过固定 SHA-256、可以执行且缓存目录可写时接入；`DIMA_CCACHE=off` 仍使用 GCC。`compiler_check=content`，不使用忽略头文件/选项的宽松命中。

## 最终门禁与交付状态

Windows 原生隔离树完成完整目标 `[74/74]` exit 0；最终共享工作区完成 `[338/338]` exit 0，覆盖 `dima_rover uorb-generated-verify mavlink-generated-verify parameter-metadata-verify logger-generated-verify`。生成、架构、ELF、签名、Factory 和未解析符号门禁均通过。共享工作区共有 480 个首方源文件、299 个参数，这些变化包含其他会话的参数/头文件整理，不属于本批新增或删减的参数与功能。

最终组合工作区的 321/321 次可缓存 Application 编译调用命中。随后使用默认自动并行度（当时可用约 706 MiB，选择 1 路）执行 `make NO_COLOR=1 upload-ready` 为 9.99 s，无编译、无完整架构扫描或重复生成；首次从完整发布目标转到上传目标仍执行一次 ELF/签名上传缓存确认。

组合制品 BIN 为 604884 B，SHA-256 为 `4637de3212f83aa402cc8f72c1b71981ee9b37df4ff4fe1340285726c91ad53b`；signed BIN 为 606059 B，SHA-256 为 `b9414502c03e7c1296a23aec7a25821e1e0761c6363ea0a6c95786b7d8d07720`；Factory HEX SHA-256 为 `793362682248c3b0b26ee3292c3de06e19f34201d346e385bfc1fbcb9b8b1d10`。不得拿组合制品与 `fa9a900` 隔离基线的差值当成本批体积或性能收益。

本批新增/修改的 Python 已通过 AST 语法检查，本批差异的 `git diff --check` 通过。没有新增测试文件、框架、fixture 或仿真，没有修改参数/msg 权威输入。上述性能验收阶段尚未提交或推送，也未访问板端串口，没有刷机或车辆动作；传输窗口比较和实际端到端上传耗时仍需授权实板验收。隔离输出与日志保留供复核，共享工作区的其他改动未回退或纳入本批归因。

## 2026-09-09 固件身份依赖修复

审查发现：HEAD 更新后，原 `dima_rover` 首次执行可能只重写身份头，继续链接并校验旧身份；第二次才重编译消费者。身份输出由 stamp 间接更新，必须在实际 Make 扫描依赖之前完成生成，不能依赖同一次扫描刷新已缓存的输出时间戳。

普通发布入口现在把身份合同纳入生成准备阶段；快速 `upload/upload-ready` 入口也先执行既有 `firmware-identity-generated`，再运行上传依赖图。输出内容相同时仍保留 mtime，避免 dry-run 虚报编译任务；成功生成后只清除该输出目录中的旧 Git 身份标记，使切回旧提交也重新生成当前合同。不增加另一套身份编码或手写版本清单。
