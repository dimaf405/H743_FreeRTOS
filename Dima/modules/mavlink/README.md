# MAVLink 协议模块

- **职责：** 独占 Application Runtime 的单路 USB CDC 数据面，承载 MAVLink v2.0 RX/TX、心跳、命令、Classic + Ext 参数、只读 Parameter/Actuator Component Metadata 与 FTP、原始 RC 通道流、空任务 mission、TIMESYNC 和 STATUSTEXT。
- **禁止事项：** 不与文本控制台共享 RX，不反向依赖 `rover/`，不直接包含 HAL/CMSIS/Core/Board 头文件；FTP 不能提供目录或任何写操作，General Metadata 只声明 Parameter 和 Actuator，不声明 Events、Peripherals、导航或 Offboard 能力；不实现执行器测试命令。
- **来源：** PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` `src/modules/mavlink/` 移植适配。

## 文件与依赖

| 文件 | 职责 |
|---|---|
| `MavlinkService.hpp/cpp` | 唯一 Console data-plane owner；模块生命周期、RX 分发、固定优先级 TX、ACK 重试、连接边沿、STATUSTEXT 和延迟 reboot |
| `MavlinkChannelState.cpp`、`lib/mavlink/mavlink_bridge.h` | 为全部 MAVLink 翻译单元提供单一 channel parser buffer 与 RX/TX sequence 状态，禁止生成 helper 在各 TU 复制状态 |
| `MavlinkSystemMessages.cpp` | PROTOCOL_VERSION、AUTOPILOT_VERSION、COMPONENT_METADATA 与 deprecated COMPONENT_INFORMATION 系统发现回复 |
| `MavlinkRcStream.cpp` | 参数刷新、原始 `input_rc` 新鲜度判定及 5 Hz `RC_CHANNELS` 流 |

| `MavlinkIdentity.hpp/cpp`、`HeartbeatPacer.hpp/cpp` | 身份、正确 capability 位图、PX4 Manual/Termination custom mode、1 Hz HEARTBEAT 和 AUTOPILOT_VERSION |
| `MavlinkCommands.hpp/cpp` | COMMAND_LONG/INT target 过滤；把 GCS `sysid/compid` 写入 `vehicle_command.source_*` |
| `MavlinkParameters.hpp/cpp`、`MavlinkParameterExt.cpp` | 主文件负责 Classic LIST/READ/SET 与 PARAM_VALUE 流；Ext 文件负责 EXT_REQUEST_READ 与 PARAM_EXT_VALUE。每次 LIST 前公开真实 RC/SER 参数及 11 项只读 QGC 关键兼容参数，在参数锁内冻结本轮 used 句柄/count/index；快照继续服务按 index 补读及快照内 READ/SET 回包，并校验端口、协议和波特率写入；未实现的模式/RTL 参数不进入协议面 |
| `MavlinkMetadataFtp.hpp/cpp` | General、Parameter、Actuator 三个 Flash 虚拟文件的只读 MAVLink FTP；支持 Open/Burst/Read/Reset/Terminate、定向回复、完整请求重复缓存、仅 EAGAIN 的 4 次有界主动 TX 重试和 10 s session 超时 |
| `MavlinkMission.hpp/cpp` | 空任务 LIST 返回 count 0，CLEAR_ALL 返回 ACCEPTED；不建立任务存储 |
| `MavlinkTimesync.hpp/cpp`、`lib/timesync/Timesync.hpp/cpp` | TIMESYNC 回传和 PX4 Timesync filter；Runtime 重建时清空 filter |

依赖仅指向 `platform/api`、`lib/mavlink`、`lib/timesync`、messages 和 middleware。ParameterService 不再读取 Console；参数在线管理通过本模块完成。

## 调度、连接与生命周期

- `MavlinkService` 在 `wq:lp_default` 以 100 Hz 运行；RX 每轮最多读取 256 bytes。
- USB `ready` false→true 时立即发送 HEARTBEAT 和 AUTOPILOT_VERSION；断线期间不推进 heartbeat pacer，正常连接后维持 1 Hz。
- Runtime start/stop 都复位 parser/channel、heartbeat、参数发送游标、Timesync、Metadata FTP session/pending/cache、ACK/reboot、连接边沿和 STATUSTEXT ID；USB 物理断开边沿丢弃 Console RX 中的旧整帧/半帧，同时复位 MAVLink parser/channel 和 FTP 状态，断线数据不能跨重连。只有 Commander Termination 等明确状态可跨 Runtime。
- TX 优先级为 COMMAND_ACK → HEARTBEAT/AUTOPILOT_VERSION → RC_CHANNELS → 单帧 Metadata FTP → 参数流 → STATUSTEXT。未解决的 FTP 发送失败会阻止本轮更低优先级流；只有明确未提交的 `EAGAIN` 主动重试，首次失败后最多重试 4 次。`ETIMEDOUT/EIO/EPIPE` 保留原回复和 session，等待 QGC 以相同 sequence 重传。新鲜且具有有效通道数的原始 RC 即使数值不变、接收机同时置 failsafe/lost 标志，也按 5 Hz 连续发送；完全无帧或样本超时时停止 RC_CHANNELS，恢复后立即重启周期流。STATUSTEXT 每轮最多 2 条，超过 5 s 的记录丢弃。
- HEARTBEAT 对正常 Rover Manual 固定发送 PX4 `custom_mode=0x00010000`；Disarmed/Armed 的 Manual `base_mode` 分别为 65/193。模式名称由 QGC 本地化，固件不发送自定义文本。内部 Termination 使用 PX4 main mode 10 并保持 Critical 状态，不伪装成 Manual。

## Commander 命令和 ACK

- `vehicle_command` 同时保存 target 和 source 地址。Commander 统一经 `answer_command()` 回应，ACK 的 target 必须等于原命令 source。
- 外部 ARM 与 RC ARM 使用同一预检：当前仅有 RC Manual 控制源，必须参数有效、Manual、RC 新鲜、摇杆居中、无 Kill/Termination 且 Flash 可上锁。拒绝映射为 `TEMPORARILY_REJECTED`，重复 ARM/DISARM 幂等 `ACCEPTED`。
- `MAV_CMD_PREFLIGHT_CALIBRATION` 只实现 QGC Radio 事务：Disarmed 下 `param4=1` 幂等进入，七参数全 0 幂等退出；Armed 返回 `TEMPORARILY_REJECTED`，其他校准类型返回 `UNSUPPORTED`。校准状态阻止 ARM、Unkill 和模式切换，Disarm/Kill/Termination 等负向安全动作继续有效。
- pending ACK 未完成时不继续消费深度 4 的 uORB ACK 队列；EAGAIN/ETIMEDOUT 最多重试 4 次，EPIPE/EIO 立即结束当前 ACK。
- Commander 只在 Disarmed 时批准 reboot mode 1/3，并通过 ACK `result_param2` 传递模式。MavlinkService 优先发 ACK；400 ms 截止时仍失败也执行已批准的复位。

## 方言与生成

方言固定为 24 条消息：21 条基础消息加上 FILE_TRANSFER_PROTOCOL、COMPONENT_METADATA 和 deprecated COMPONENT_INFORMATION 回退。COMPONENT_INFORMATION_BASIC 与 SERVO_OUTPUT_RAW 不进入裁剪方言。

`AUTOPILOT_VERSION.capabilities` 声明 `PARAM_FLOAT | PARAM_ENCODE_BYTEWISE | FTP | COMMAND_INT | MAVLINK2`。FTP capability 只对应下述只读 Component Metadata 虚拟文件，不代表通用文件系统。

固件保留 QGC 5.0.x Radio 页面发现和通道映射所需的 `RC_CHANNELS`、`RC_CHAN_CNT`、Roll/Pitch/Throttle/Yaw 及 Flaps/Aux 参数；当前目标是连接后能正常打开并显示通道映射，不把完整四轴校准流程作为本阶段交付能力。板级编号固定为 USB `SERIAL0` 加八路直接对应 MCU 外设号的 `SERIAL1..8`；参数列表公开成对的 `SERIALx_BAUD`/`SERIALx_FUNCTION`。当前 Function 只允许 Disabled/RC Input，默认 `SERIAL6=USART6/PC7` 为 RC；不存在旧板级键或迁移参数。SBUS 接管唯一 RC 端口时临时切换为 100000/8E2/RXINV，停止后恢复普通串口配置。串口 MAVLink、SERIAL_CONTROL、GPS/RS485 数据服务、实际 Sensor/RTL/Battery、非 Manual 模式、`PARAM_MAP_RC`、Spektrum/CRSF Bind 和 Copy Trims 均未实现。

Classic `PARAM_VALUE` 不携带 group、description 或 enum。构建把当前全部 203 项参数包装为 QGC version 1 Parameter JSON，并从同一参数目录生成六路 `PWM_Sn_FUNC/MIN/CENT/MAX/REV` 的 Actuator JSON；`component_general.json.xz` 声明 `COMP_METADATA_TYPE_PARAMETER=1` 和 `COMP_METADATA_TYPE_ACTUATORS=5`。QGC 通过 `MAV_CMD_REQUEST_MESSAGE(397)` 获取现代 URI/CRC，必要时用 395 回退，再经只读 MAVLink FTP 下载三份文件。当前 QGC Actuators 页面已确认可分配和编辑六路输出；这是有意偏离 pinned PX4 mixer 要求的 Dima output-only 适配，结构门禁固定为非空 `outputs_v1` 与空 `mixer_v1.config`。MotorRight/MotorLeft 标记为 `exclude-from-actuator-testing`，固件不处理 `MAV_CMD_ACTUATOR_TEST`，也不提供 `SERVO_OUTPUT_RAW` 输出遥测。

生成输入为 pinned `common.xml/standard.xml/minimal.xml`、`mavlink.lock.json`、`build_trimmed_dialect.py`、公开参数 JSON 和 `generate_parameter_metadata.py`。pymavlink 固定为 commit `fcaa2c7d25e3169dc66155929c338487941555e9`（2.4.47）；MAVLink 头与 Metadata JSON/XZ/Flash 数组只进入 `build/generated/`，由 `make dima_rover` 从无生成物状态完整生成和校验，不进入源码库。
