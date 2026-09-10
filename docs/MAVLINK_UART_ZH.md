# H743 USB / UART MAVLink 双链路

实现基线为 PX4 v1.17.0 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。UART 接透明数传电台或 USB 转串口，使用标准 MAVLink 2、8N1。USB 使用 Config 策略，UART 使用 Normal 策略；两者报告同一系统身份和硬件 UID，均提供已有参数、命令/模式、Mission、Metadata FTP 和日志服务。

## 配置入口

默认没有 `SERIALx_FUNCTION=3`，不预占数传 UART。GPS/SBUS 原默认配置保持，SERIAL5 继续为空号。选择实际接线的空闲串口，设置 `SERIALx_FUNCTION=3 (MAVLink)` 即可；参数写入顺序不限。将新端口设为 MAVLink 会通过参数原子事务关闭旧 MAVLink owner，始终只有一个 UART 数传 owner。`SERIALx_BAUD` 显式为非零值时直接以该速率运行；保持 `0=Auto` 时帧格式仍为 8N1，波特率由固件按常见数传速率逐档自动探测（9600..921600 共八档，每档观察 4 s，窗口内收到不少于 2 帧完整 MAVLink 帧即锁定；锁定速率不回写参数，重配置或重启后重新探测）。参数仍只有一份权威目录，USB 和 UART 的参数读取快照及下载游标分别保存。

`MAV_0_RATE` 只控制本 UART 实例，默认 0：自动预算为 `baud / 20 B/s`，即 8N1 理论上限 `baud / 10` 的一半。正值选择显式预算，并受理论线路带宽约束。沿用 PX4 流速率调整原则，从预算中扣除实际事务占用，再按周期流需求求倍率，约束在 0.05～1；队列拥塞进一步降低倍率。该预算用于降低周期流频率，事务可使用空闲线路容量。低波特率或连续参数/日志传输时，实际周期频率会下降。

Normal 周期策略由 `Dima/modules/mavlink/mavlink_runtime.yaml` 经正式生成器产生：

| 消息 | UART 默认频率 |
|---|---:|
| HEARTBEAT、SYS_STATUS、LOCAL_POSITION_NED | 1 Hz |
| GPS_RAW_INT、GLOBAL_POSITION_INT、RC_CHANNELS | 5 Hz |
| ATTITUDE | 15 Hz |
| ESTIMATOR_STATUS、CURRENT_MODE | 0.5 Hz |
| HIGHRES_IMU、SCALED_IMU | 关闭，可按需请求或设置间隔 |

USB 默认频率保持原 Config 策略。SET/GET_MESSAGE_INTERVAL 只作用于请求链路；0 恢复该链路策略，GET 回报原始请求间隔，带宽降频不覆盖请求值。现有 Mission 进度等事务/状态消息仍使用原业务调度。

## 驱动与调度边界

`MavlinkService` 是唯一生命周期及协议 WorkQueue owner，内部固定两个 `MavlinkEndpoint`，分别使用 `MAVLINK_COMM_0` 和 `MAVLINK_COMM_1`。编码与解析全部经过正式 mavgen 通道接口。parser、TX 序号、参数传输、Mission 协议、FTP、响应队列和周期配置彼此隔离。没有跨链路转发，也没有新的 Offboard 模式。

UART 全双工实现由两个固定资源实例复用。原端点保持 DMA1 Stream3 RX / IT TX；数传端点使用 DMA1 Stream4 RX、Stream5 TX，避开 SPI/SBUS 使用的 Stream0～2。数传 RX DMA 为 1024 bytes、软件 RX Ring 为 4096 bytes、TX staging 为 512 bytes；DMA 缓冲位于现有 `.dima_dma` 非缓存区域。发送先复制到端点 staging，HAL 的 UART TC 回调才释放在途所有权，DMA TC 不作为线路完成证据。

每条链路使用 8 槽固定完整帧 FIFO。RX 派发前预留回应容量，无法接纳的日志请求保留在 parser 中，避免因暂时拥塞重复应用参数写入或命令。ACK/心跳先于新后台帧入队；已编码帧保持 FIFO 线序。UART 后台最多预排一帧，另允许一帧已在 staging 中发送；UART 不同步等待串行线路。USB 仍支持一次最多 16 个完整日志帧的 CDC 合并，各写入共用每轮 5 ms 总等待预算。

数传端口/波特率改变时，appMain 先请求协议层停止接纳 UART 新业务并排空已接纳的 TX，再申请现有未解锁维护票据。等待采用非阻塞状态机，继续主循环与喂狗。线路排空期限按在途/排队字节、8N1 的 10 bit/byte 和当前波特率计算，含 200 ms 调度余量。排空失败保留当前配置；新驱动启动失败恢复旧配置，并在候选未被更新时原子恢复旧参数快照。USB 始终保留配置入口。

普通重启 ACK 返回请求来源链路，提交成功与物理发送完成分开记录。ACK 未获得物理完成证据而超过推导期限时取消该次重启并报告，防止低波特率截断 ACK。`param1=3` 的 MCUboot Recovery 只允许 USB，固件上传继续使用 USB。

## 共享业务仲裁

- Commander 命令通过共享入口发布一次；记录来源链路和连接代次，最终 ACK 只回原链路。全局确认槽忙时，新冲突命令返回 `TEMPORARILY_REJECTED`。查询/间隔等链路本地请求直接回应，不改 uORB 消息格式。
- Mission 写入使用原单事务 token；另一链路的上传、清空或设置当前项冲突返回 `MAV_MISSION_DENIED`。断链撤销未完成接收；已提交存储的完成结果仍被消费，旧最终 ACK 不进入新会话。
- 日志列表、下载与擦除共享单 reader 租约。非 owner 请求得到限速 busy 文本；非 owner 的 END/ERASE 不能关闭另一链路 reader，也不伪造空列表。有效请求、预读或发送进展续期，无进展 5 秒由 storage worker 关闭/释放，队列满时也保留到期唤醒。END 后租约最长保留 5 秒，支持同链路紧随的 LIST。
- `STORAGE_INFORMATION` 是独立只读请求；另一链路持有日志租约时仍可查询。END/ERASE 保留已接纳的容量查询及其响应。
- 所有 SD 文件操作仍在 `wq:storage`，增加的是固定 WorkItem/链路状态，没有新增 RTOS 任务、heap 容量或任务栈容量。

UART RX 溢出、UART/DMA 错误和接收恢复有累计计数。故障仅恢复该 UART，并重置该协议通道；持续错误最多每秒输出一次 `UART MAVLink drop=... rxerr=... txerr=... recover=... fail=...`。UART running 只表示端点配置成功，不代表无线电另一端或 GCS 已连接。无线静默由现有 Mission/FTP/日志会话超时管理，GCS 失联安全策略保持现有实现。

## 软件验收记录

源码基准为实现前 `93cdae6`，Linux 原生构建输出到 `build-linux/`。正式入口：

```sh
make -j4 NO_COLOR=1 parameter-generated mavlink-generated
make -j4 NO_COLOR=1 mavlink-generated-verify parameter-metadata-verify
make -j4 NO_COLOR=1 dima_rover
make -j4 NO_COLOR=1 verify
```

最终构建、显式检查和 ELF 资源数值见本文末尾记录。未创建或修改测试文件、框架、测试基础设施。软件生成/编译/链接结果不代表目标板传输结果。

## 手动板端验收

UART 和主机 COM 号由使用者按实际接线选择，本次未连接板端串口，也未刷写。以下项目均待实机验证；日志/参数和 QGC tlog 请随记录保存。

| 场景 | 操作与通过标准 |
|---|---|
| 默认配置 | 保留已有 GPS/SBUS 默认，确认 UART 数传未启用；USB 可读参数、心跳和 Metadata。 |
| 三种连接组合 | USB 单独、UART 单独、同时连接 QGC；系统 ID、component ID、AUTOPILOT_VERSION.uid 一致，业务回应经原请求链路返回。 |
| 三档波特率 | 在 57600、115200、921600 各完成参数下载/修改、模式命令、Mission 上传回读、Metadata 获取及完整日志下载；记录耗时、错误计数和心跳最大间隔。 |
| 链路隔离 | 为两链路设不同消息间隔，分别开启参数/FTP 会话；拔 USB 或从 USB 修改 UART 波特率，另一链路的会话及间隔保持；重连的链路恢复自己的默认策略。 |
| Mission 冲突 | 一链路上传未完成时另一链路上传/清空，后者得到 DENIED；先发事务内容、数量和回读一致。 |
| 日志租约 | 一链路下载时另一链路 LIST/DATA/ERASE 得 busy；非 owner END 无效。两侧容量查询均可返回。owner 停止有效请求且无传输进展 5 秒后，另一链路可取得租约。 |
| 低速拥塞 | 连续日志/参数传输时请求模式/心跳；UART 等待有界，USB 仍可配置，不能引发维护超时或看门狗重启。记录实际周期降频，不以默认频率作为满载保证。 |
| 自身切换 | 通过 UART 修改自身 BAUD/迁移到另一空闲端口；旧 UART 先收到完整 PARAM_VALUE，新配置随后启用。非法 BAUD/owner 不替换有效配置，失败应恢复旧端口。 |
| 错误恢复 | 错速、噪声/过量 RX 或调试器注入 UART/DMA 故障，观察累计诊断；恢复接线/波特率后可重建本链路，USB 会话继续有效。 |
| 重启与 Recovery | 从 UART/USB 分别请求普通重启，示波器或串口采集确认 ACK 最后停止位先于复位；UART Recovery 请求拒绝，USB Recovery/原上传流程保持。 |
| 文件完整性 | 对同一已关闭日志经 USB/UART 下载，比较文件大小及 SHA-256；二者应与 SD 原文件一致。 |

建议每档至少记录固件 SHA-256、SERIALx 参数、MAV_0_RATE、QGC 版本、连接组合、文件大小/哈希、用时、错误计数及结果。低速无线实际吞吐还取决于电台空口配置和缓存；首版不提供 AT 管理或硬件流控。

## 2026-09-10 软件验证结果

- `parameter-generated`、`mavlink-generated` 完成；另用同一正式参数生成器在临时输出目录重生成，8 个文件与构建输入逐字节一致。生成目录为 299 项参数，包含默认 0 的 MAV_0_RATE；全部串口 Function 枚举含 3=MAVLink，默认均未选 3，SERIAL5 不存在。
- `mavlink-generated-verify`、`parameter-metadata-verify` 通过。原锁定 MAVLink XML/mavgen 的两条 bitmask 提示仍来自上游输入，生成一致性检查成功。
- Linux 原生 `make -j4 NO_COLOR=1 dima_rover` 通过；最终 `make -j4 NO_COLOR=1 verify` 通过，架构覆盖 489 个首方源文件，Logger/Metadata、应用 ELF 布局、签名、Factory/MCUboot 布局及 watchdog 链均通过。
- Application 和 MCUboot 的 `nm -u` 均为空。ELF 包含 DMA1 Stream0～5 中断及 UART TC 回调；第二端点 RX/TX DMA 分别位于 `0x30040a40` / `0x30040840`，长度为 1024 / 512 bytes，软件 RX Ring 为 4096 bytes。通道 buffer/status 各有两份。

| ELF 项目 | 实现前 | 最终 | 增量 |
|---|---:|---:|---:|
| `.data` | 2,692 B | 2,700 B | 8 B |
| `.bss` | 172,832 B | 197,952 B | 25,120 B |
| `.dima_dma` | 6,208 B | 7,744 B | 1,536 B |
| 其余 RAM 段 | 保持 | 保持 | 0 B |
| 新增 RAM 段合计 | — | — | **26,664 B（26.039 KiB）** |

新增静态 RAM 低于 32 KiB，余量 6,104 B。`.dima_heap` 保持 262,144 B，`.dima_task_pool` 保持 49,152 B，DTCM 热点对象段及主栈预留均未扩大。链接布局按对齐区间统计的 SRAM 总占用为 560,896 B，DTCM 静态占用为 60,896 B，Flash 为 604,644 B；区域对齐统计与上述段载荷之和有少量差异。

本次签名固件为 `build-linux/H743_FreeRTOS_signed.bin`，605,818 bytes，SHA-256：

```text
804423d4589b723581341cb11387c13a6b85ed82a47695a5ad5424fb316afdd5
```

对应 ELF SHA-256 为 `37523153da7c4987f8d2867f825ffcfe339a3dac374d70415332bb11dd3ce083`。板端与 QGC 动态项目保持待验，没有将软件检查计为实机通过。

## 2026-09-10 Auto 波特率与顺序自由化修订

板端实测暴露两项合同缺陷，本节记录修订：

1. **运行期串口重配死循环**：SBUS 停止后 timestamped 端点仍保留“正常 UART”接管快照（`normal_configuration_valid_`），`configure_line` 的端点所有权门禁因此拒绝所有端口应用与回滚，`SerialConfig::reconfigure()` 每 ~1.4 s 失败重试。修复为候选校验通过后先调用 `SerialPorts::reset_configuration()` 释放快照（RC 链此刻已停止，`SbusRc::start()` 重启时重新 `configure()` 重建），释放失败以独立报错退出，不再落入误导性的 `serial baud rollback failed`。
2. **Auto 语义与参数顺序**：`SERIALx_BAUD=0 (Auto)` 对所有非 SBUS 设备合法——帧格式固定 8N1，仅波特率由设备驱动自动探测（GPS 由 UM982 八档扫描收敛 460800；MAVLink 由服务层扫描 9600/19200/38400/57600/115200/230400/460800/921600，每档 4 s 窗口、≥2 帧完整 MAVLink 帧锁定，锁定速率不回写参数）。`PARAM_SET` 路径删除“FUNCTION=3 必须先有非零 BAUD”的顺序门槛，FUNCTION 与 BAUD 任意顺序写入均被接受；扫描在 Auto 模式下于 `wq:lp_default` 内运行，不新增任务与堆分配。参数与串口 YAML 描述同步修订。

Linux 原生 `make -j4 NO_COLOR=1 dima_rover` 与 `make -j4 NO_COLOR=1 parameter-metadata-verify` 通过后本节记录最终数值。Auto 扫描锁定时限、误锁恢复与电台侧空口行为仍属 BOARD PENDING。
