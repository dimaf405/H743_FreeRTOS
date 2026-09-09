# IMU 遥测运行后停更：QGC 证据与修复

日期：2026-09-08。用户现场：无 SD 卡，QGC 保持连接，仍可修改参数；姿态、IMU、位置与估计器消息停止更新。

## 现场证据

直接复制 QGC 正在使用的临时日志，未关闭 QGC、占用 COM10、复位或刷写设备：

- 原始文件：C:\Users\master\AppData\Local\Temp\FlightDatapZaAWO.mavlink。
- 固定副本：C:\Users\master\AppData\Local\Temp\h743-telemetry-stall-cb035a24\qgc_live_snapshot.tlog。
- 副本大小：2976774 bytes。
- SHA-256：902fd77ee594216c43ec2b4c21f4e19a3bc09c6b49290c2c1a1099fd26810c98。
- 记录范围：北京时间 14:22:24.405–15:38:03.737。
- 解析使用项目锁定的 pymavlink；其自带 common Python 方言未覆盖部分新消息，本次判定只使用正常解码的 IMU、SYS_STATUS、姿态/位置/估计器、STATUSTEXT 和 AUTOPILOT_VERSION，不手写缺失消息 codec。

| 时间 | 观测 |
|---|---|
| 14:22:24.405 | IMU 开始出现在连接日志，样本时间为上电 16.122031 s |
| 14:27:04.645 | SYS_STATUS.errors_count1=19952，IMU health=3，CPU load=358 |
| 14:27:05.515 | 最后一条 HIGHRES_IMU，样本时间为上电 297.333994 s |
| 14:27:05.545 | 最后一条 SCALED_IMU |
| 14:27:05.654 | errors_count1=20008，IMU health=0 |
| 14:27:06.335–14:27:06.435 | 估计器状态、姿态和本地位置依次停止发送 |
| 14:27:06.654 | errors_count1=20060，CPU load=237 |
| 15:38:03.737 | SYS_STATUS 仍收到；errors_count1 饱和为 65535，health=0 |

原始错误合计在 IMU 停更后仍增加，说明 raw sensor Topic 仍在推进。CPU 降低对应前端/EKF 工作停止，不能作为 SD 或 CPU 优化收益。无 SD 卡现场没有正在执行日志块 IDMA 传输的证据。

板端 AUTOPILOT_VERSION 的 flight_custom_version 按现有字节序对应 Git 标识 3ac342f808e847fe；本次开始时当前工作区 HEAD 为 121b8885357d377b5365413067d1300635f73b3b。旧构建快照与当前源码的 ICM42688P.cpp、ICM42688PFifo.cpp 和 DataValidator.cpp 逐字一致，故本次门限及驱动路径分析有对应源码依据；不能仅用旧板端 Git 标识代替完整工作区快照。

## 已确诊的停更机制

ICM 将同一 sensor_error_count 同时写入 sensor_accel.error_count 与 sensor_gyro.error_count。SYS_STATUS.errors_count1 是两者之和，并饱和到 UINT16_MAX。

DataValidator 对每路 error_count > 10000 设置 StreamFailureHighErrorCount。此次合计 20008 表明至少一路已超过 10000；同批次两路相同时为各 10004，与停止更新时刻吻合。遥测分次复制两路数据，不把合计当作原子双通道快照。VehicleImu 健康门禁随后清积分器并停止发布 vehicle_imu；HIGHRES_IMU/SCALED_IMU 没有新输出，EKF 输入也中断。姿态、位置与估计器遥测在原有新鲜度窗口结束后停止。通信任务继续发送 HEARTBEAT/SYS_STATUS 和处理参数。

旧日志没有各类 FIFO/总线错误的分项快照，因此可以确认“累计错误门限触发前端抑制”，不能仅凭合计断言全部 10004 次都来自空 FIFO。

源码另确认一处会推动此门限的计错缺陷：一次合法空 FIFO 读取返回 false，同时增加 fifo_empty 与外层 transfer_failures，而 sensor_error_count 又把二者相加。一次空轮询因而使每路 error_count 增加 2，SYS_STATUS 合计增加 4。水位通知与 DMA 完成通知交叠时，空读本身不等于 SPI/DMA 故障。

## 实现变更

- FIFO 处理显式返回 Published、NoData、Failed。暂时为空只增加 fifo_empty 诊断，不增加传输失败或发布错误计数，也不消退先前真实失败。
- 独立保留 100 ms 无有效数据期限：进入 Running 建立起始窗口，之后只有成功发布双 Topic 才推进时间；连续空读越过期限计一次传输故障并重启采集。永久无数据仍会恢复，不把空读当作有效样本。
- DMA 5 ms 超时、FIFO 溢出、非法包、寄存器异常、错误密度和累计错误门限均保留；没有清零历史错误或提高健康门限来隐藏故障。
- MAVLink 非实时 owner 在 IMU 健康丢失/恢复边沿发送摘要；丢失信息包含 raw_ms、out_ms 和各路 err，USB 重连重报未恢复的异常。实时传感器队列的格式化保护不放开。
- SD IDMA、CPU 统计、参数定义和消息契约不属于此次修改；并行任务修改继续保留。没有新增测试文件、框架、模拟环境或测试接口。

## 验收边界

真实旧固件的停更时间、门限、错误继续增长和通信存活已由本次 QGC 日志确认。源码审查覆盖空读不计错、不伪造恢复、连续无数据恢复、真实传输错误、有效发布推进窗口及故障摘要重连。

Windows 原生 make -j4 NO_COLOR=1 dima_rover 与 make intellisense 已通过。架构门禁为 454 个第一方源码文件；IDE 数据库含 356 条命令、335 个源文件。ELF 已确认 run_fifo、process_fifo_transfer、report_imu_fault 以及新诊断文字进入镜像，DataValidator 原门限逻辑保留；签名和 factory 布局验证通过，应用/MCUboot 无未解析符号。D1 SD DMA 段仍为 0x24060000、8192 bytes、NOLOAD。

本次快照的签名 BIN 为 621623 bytes，SHA-256 为 77289baa24aa7fee40a4c92bc978e9b4d9ae4560dcf837739ce2311ddbaaca1c；未签名 BIN 为 620448 bytes，SHA-256 为 b35111690805ca43bfef521c99a30f3f3e31d9a945c482883e07e8b67e5c9313。两者及 ELF/map/factory、源文件哈希清单保存在故障快照目录的 fixed_firmware 子目录。构建包含共享工作区中的其他并行修改，整体大小变化不全部归因于本修复。

新固件尚未刷入当前设备。修复后的长时间运行、真实 FIFO 计数分布及无数据恢复需要板端复验；静态/构建结果不替代该结论。

