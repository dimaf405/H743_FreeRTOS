# 头文件运行期实现分离记录

日期：2026-09-09。按用户要求修正 GpsErrorCounter，并扫描、迁移项目维护代码中同类头文件实现。

## 范围与结果

本批通过 C++ 语法树扫描项目维护的 Dima、Boards、Core、Bootloader、USB_DEVICE 头文件。将 64 个头文件中的 426 处普通运行期定义移入对应 .cpp/.c，包括普通访问器、构造逻辑及仅用于运行期的本地 constexpr 辅助函数；另外将 OutputPredictor 的私有平方副本改为调用既有同签名同表达式的 math::Utilities::sq。

GpsErrorCounter.hpp 现在只保留声明与计数状态；会话基线、UART 增量、饱和累加和访问器均位于 GpsErrorCounter.cpp。新增 26 个源文件，分别进入现有 Make 所有者或正式目录发现闭包；平台 API 的实现由 platform/common 提供，FreeRTOS、STM32 和板级私有逻辑仍由原后端持有。

不移动通用模板必需的可见定义，不手改正式生成文件。C++ 编译期值接口以及 =default/=delete 保留类型和语言语义，HAL/CMSIS/第三方原件继续按来源维护。按用户后续补充约束，仅含单行 `return` 语句的函数允许保留在头文件中。本次已外移的简单访问器仍保留在源文件，符合该约束；外移数量记录实际变更，不代表这些访问器今后必须外移。

最终复扫覆盖 247 个头文件，普通运行期定义剩余 0。保留类别：

- template definition visibility：571 处。
- compile-time value/compatibility contract：14 处。
- authoritative generated code：14 处。

## 等价边界与依赖修正

- 426 个迁移函数的主体 token 与构造初始化表达式已核对；条件编译边界、成员布局、默认参数和原有错误/回滚策略保留。
- 原先由 auto 函数体推导的返回接口改为显式声明原成员类型；继续消费现有生成的 aid-source 结构，不新增消息字段或类型清单。StateResets 仅补前置声明。
- GNSS 状态 union、参数层容量和 BiasEstimator 的类型来源改为显式限定；AtomicBitset/TrajMath 补齐声明本身需要的基础类型依赖，不改变算法。
- MessageFormatReader 的压缩窗口与 decoder 静态断言仍放在头中，使所有消费者继续接受编译期布局核对。
- Board 启动请求提供统一 C ABI，硬件 include 留在板级源文件；MAVLink channel 状态仍由同一 owner 提供。
- OutputPredictor 的两个平方调用改用 math::Utilities::sq，float 签名与 var*var 表达式相同。
- 没有新增或修改测试文件、测试框架、模拟环境或测试专用接口。语法解析工具仅存在于本机临时工作目录，不加入产品或构建依赖。

## 构建与并发边界

基线构建因共享 HEAD 更新触发一次依赖重算，原命令重跑后通过，保存 BIN 为 601940 B。执行期间其他会话继续修改 DroneCAN 参数和构建缓存/依赖规则；这些变更完整保留，参数数量从 300 变为 299 不属于本批。因此不把前后整树 Flash 差值当成本批独立收益或开销。

首次落地后补齐新编译单元的私有 include、自包含类型声明和显式返回类型。正式 Windows 验收已完成：

- make -j4 NO_COLOR=1 dima_rover uorb-generated-verify mavlink-generated-verify parameter-metadata-verify logger-generated-verify：67/67，exit 0。
- 架构通过 480 个第一方源文件，应用和 MCUboot 未解析符号为空，ELF/向量/内存布局、签名及 Factory 镜像检查通过。
- make NO_COLOR=1 intellisense：383 条命令、361 个源文件；新增 26 个实现文件均出现在数据库中。
- 全库语法复扫普通运行期定义剩余 0；426 个迁移主体及构造初始化 token 核对通过，没有遗漏额外 lambda 或函数 try-body。
- DTCM 任务栈池、CPU 对象段、D1 heap/日志/SD 缓冲的地址和大小与基线一致；init_array 仍为 16 B，四个初始化入口及顺序保持。
- 运行期实现外移会改变内联机会，板端 CPU/时序结果尚未测量。

| 最终冻结产物 | B | SHA-256 |
|---|---:|---|
| H743_FreeRTOS.bin | 604884 | 4637de3212f83aa402cc8f72c1b71981ee9b37df4ff4fe1340285726c91ad53b |
| H743_FreeRTOS_signed.bin | 606059 | c24be094621f3646a5ce430f4807aa666b73f18c5345db817782a17ece18f1e8 |

最终 Flash load span 为 604884 B / 782336 B（77.32%），链接余量 177452 B（173.29 KiB）。本轮包含并发业务/构建变化，不将与旧基线的 2944 B 差值直接认定为纯迁移成本。冻结产物与源码哈希见临时目录的 artifact_manifest.json 和 after 子目录。

全部原文、迁移记录、语法复扫与构建日志：C:\Users\master\AppData\Local\Temp\h743-header-split-pxm9sz_m。原始 tools/upstream 快照未改，适配边界已记录到 DIMA_SOURCE_MANIFEST.md。
板端启动、传感器、参数生命周期、USB/QGC 与任务时序未实测，未刷写设备。

## 分批提交前复核

按主题分批提交前，保留上述冻结产物，重新在 Windows 原生工作区执行同一组正式目标：7/7，exit 0（增量验收，无源码重编译）。架构仍为 480 个项目源文件，四项生成校验、ELF、签名与 Factory 检查通过；IDE 再生成结果仍为 383 条命令、361 个源文件。

本次待提交源码与迁移主体验收快照一致，新增的单行 `return` 例外仅调整仓库规则与说明。当前 BIN 仍为 604884 B，SHA-256 与上表一致；当前 signed BIN 为 606059 B，SHA-256 为 `b9414502c03e7c1296a23aec7a25821e1e0761c6363ea0a6c95786b7d8d07720`。上述冻结签名与当前签名分别记录，不混用制品身份。

本次分批只重组 Git 提交，工作区源码保持不变。正式构建证据针对最终组合工作区，各中间提交未单独重新构建。逐批暂存补丁、提交范围及最新验收日志保存在 `C:\Users\master\AppData\Local\Temp\h743-batch-commits-suakqg7z`。没有推送或刷机，板端和性能验证边界不变。

## 迁移清单

下表由本次语法树迁移记录生成；数量仅统计外移定义，OutputPredictor 私有平方复用另计 1 处。

| 头文件 | 外移数量 | 实现文件 |
|---|---:|---|
| Boards/H743/Inc/dima_boot_request.h | 6 | Boards/H743/Src/dima_boot_request.c |
| Bootloader/Inc/flash_map_backend/flash_map_backend.h | 6 | Bootloader/Src/flash_map_backend.c |
| Bootloader/Inc/os/endian.h | 1 | Bootloader/Src/endian.c |
| Dima/adapters/mavlink/MavlinkBridge.h | 2 | Dima/adapters/mavlink/MavlinkBridge.cpp |
| Dima/drivers/gps/um982/GpsErrorCounter.hpp | 8 | Dima/drivers/gps/um982/GpsErrorCounter.cpp |
| Dima/drivers/imu/icm42688p/ICM42688P.hpp | 1 | Dima/drivers/imu/icm42688p/ICM42688P.cpp |
| Dima/drivers/imu/icm42688p/ICM42688PFifoAlgorithms.hpp | 7 | Dima/drivers/imu/icm42688p/ICM42688PFifoAlgorithms.cpp |
| Dima/drivers/imu/icm42688p/ICM42688PRegisters.hpp | 4 | Dima/drivers/imu/icm42688p/ICM42688PRegisters.cpp |
| Dima/drivers/magnetometer/dronecan_mag2/DroneCanMag2.hpp | 1 | Dima/drivers/magnetometer/dronecan_mag2/DroneCanMag2.cpp |
| Dima/drivers/rc/sbus/SbusProtocol.hpp | 1 | Dima/drivers/rc/sbus/SbusProtocol.cpp |
| Dima/lib/dronecan/DroneCanNode.hpp | 4 | Dima/lib/dronecan/DroneCanNode.cpp |
| Dima/lib/dronecan/TransferIdTracker.hpp | 6 | Dima/lib/dronecan/TransferIdTracker.cpp |
| Dima/lib/ekf2/EKF/aid_sources/ZeroVelocityUpdate.hpp | 1 | Dima/lib/ekf2/EKF/aid_sources/ZeroVelocityUpdate.cpp |
| Dima/lib/ekf2/EKF/aid_sources/gnss/gnss_checks.hpp | 13 | Dima/lib/ekf2/EKF/aid_sources/gnss/gnss_checks.cpp |
| Dima/lib/ekf2/EKF/bias_estimator/bias_estimator.hpp | 11 | Dima/lib/ekf2/EKF/bias_estimator/bias_estimator.cpp |
| Dima/lib/ekf2/EKF/bias_estimator/height_bias_estimator.hpp | 5 | Dima/lib/ekf2/EKF/bias_estimator/height_bias_estimator.cpp |
| Dima/lib/ekf2/EKF/bias_estimator/position_bias_estimator.hpp | 15 | Dima/lib/ekf2/EKF/bias_estimator/position_bias_estimator.cpp |
| Dima/lib/ekf2/EKF/ekf.h | 93 | Dima/lib/ekf2/EKF/ekf.cpp |
| Dima/lib/ekf2/EKF/estimator_interface.h | 50 | Dima/lib/ekf2/EKF/estimator_interface.cpp |
| Dima/lib/ekf2/EKF/imu_down_sampler/imu_down_sampler.hpp | 1 | Dima/lib/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp |
| Dima/lib/ekf2/EKF/output_predictor/output_predictor.h | 12 | Dima/lib/ekf2/EKF/output_predictor/output_predictor.cpp |
| Dima/lib/ekf2/EKF/yaw_estimator/EKFGSF_yaw.h | 6 | Dima/lib/ekf2/EKF/yaw_estimator/EKFGSF_yaw.cpp |
| Dima/lib/geo/geo.h | 6 | Dima/lib/geo/geo.cpp |
| Dima/lib/lat_lon_alt/lat_lon_alt.hpp | 16 | Dima/lib/lat_lon_alt/lat_lon_alt.cpp |
| Dima/lib/mathlib/math/Functions.hpp | 4 | Dima/lib/mathlib/math/Functions.cpp |
| Dima/lib/mathlib/math/Limits.hpp | 2 | Dima/lib/mathlib/math/Limits.cpp |
| Dima/lib/mathlib/math/TrajMath.hpp | 4 | Dima/lib/mathlib/math/TrajMath.cpp |
| Dima/lib/matrix/matrix/helper_functions.hpp | 2 | Dima/lib/matrix/matrix/helper_functions.cpp |
| Dima/lib/rover/CalibrationIdentification.hpp | 5 | Dima/lib/rover/CalibrationIdentification.cpp |
| Dima/lib/rover/CalibrationResponse.hpp | 8 | Dima/lib/rover/CalibrationResponse.cpp |
| Dima/lib/rover/RoverControl.hpp | 1 | Dima/lib/rover/RoverControl.cpp |
| Dima/lib/sensors/calibration/SensorCalibrationAlgorithms.hpp | 1 | Dima/lib/sensors/calibration/SensorCalibrationAlgorithms.cpp |
| Dima/lib/sensors/validation/DataValidator.hpp | 3 | Dima/lib/sensors/validation/DataValidator.cpp |
| Dima/lib/sensors/validation/SensorValidityAlgorithms.hpp | 2 | Dima/lib/sensors/validation/SensorValidityAlgorithms.cpp |
| Dima/lib/serial/SerialPortAssignments.hpp | 4 | Dima/lib/serial/SerialPortAssignments.cpp |
| Dima/middleware/parameters/ConstLayer.h | 8 | Dima/middleware/parameters/ConstLayer.cpp |
| Dima/middleware/parameters/Crc32.hpp | 2 | Dima/middleware/parameters/Crc32.cpp |
| Dima/middleware/parameters/DynamicSparseLayer.h | 16 | Dima/middleware/parameters/DynamicSparseLayer.cpp |
| Dima/middleware/parameters/ParamLayer.h | 1 | Dima/middleware/parameters/ParamLayer.cpp |
| Dima/middleware/parameters/atomic_transaction.h | 4 | Dima/middleware/parameters/atomic_transaction.cpp |
| Dima/middleware/parameters/autosave.h | 2 | Dima/middleware/parameters/autosave.cpp |
| Dima/middleware/parameters/flashfs.h | 1 | Dima/middleware/parameters/flashfs.cpp |
| Dima/middleware/parameters/param.h | 1 | Dima/middleware/parameters/param.cpp |
| Dima/middleware/rover/RoverModeContract.hpp | 9 | Dima/middleware/rover/RoverModeContract.cpp |
| Dima/middleware/uORB/uORB.hpp | 3 | Dima/middleware/uORB/uORB.cpp |
| Dima/middleware/uORB/uORBMessageFields.hpp | 6 | Dima/middleware/uORB/uORBMessageFields.cpp |
| Dima/middleware/work_queue/WorkQueue.hpp | 2 | Dima/middleware/work_queue/WorkQueue.cpp |
| Dima/modules/mavlink/MavlinkIdentity.hpp | 7 | Dima/modules/mavlink/MavlinkIdentity.cpp |
| Dima/modules/mission/MissionRepository.hpp | 4 | Dima/modules/mission/MissionRepository.cpp |
| Dima/modules/safety/Commander.hpp | 1 | Dima/modules/safety/Commander.cpp |
| Dima/modules/sensors/imu/VehicleImu.hpp | 1 | Dima/modules/sensors/imu/VehicleImu.cpp |
| Dima/modules/sensors/imu/VehicleImuAlgorithms.hpp | 20 | Dima/modules/sensors/imu/VehicleImuAlgorithms.cpp |
| Dima/modules/sensors/magnetometer/VehicleMagnetometer.hpp | 1 | Dima/modules/sensors/magnetometer/VehicleMagnetometer.cpp |
| Dima/platform/api/Execution.hpp | 1 | Dima/platform/common/Execution.cpp |
| Dima/platform/api/Flash.hpp | 2 | Dima/platform/common/Flash.cpp |
| Dima/platform/api/Memory.hpp | 1 | Dima/platform/common/Memory.cpp |
| Dima/platform/api/PlatformTypes.hpp | 1 | Dima/platform/common/PlatformTypes.cpp |
| Dima/platform/api/Synchronization.hpp | 4 | Dima/platform/common/Synchronization.cpp |
| Dima/platform/api/Time.hpp | 3 | Dima/platform/common/Time.cpp |
| Dima/platform/freertos/BackendTimeout.hpp | 1 | Dima/platform/freertos/BackendTimeout.cpp |
| Dima/platform/stm32h7/spi/SpiClockDivider.hpp | 2 | Dima/platform/stm32h7/spi/SpiClockDivider.cpp |
| Dima/rover/control/RoverControlValidation.hpp | 2 | Dima/rover/control/RoverControlValidation.cpp |
| Dima/rover/modes/auto_calibration/AutoCalibrationMode.hpp | 1 | Dima/rover/modes/auto_calibration/AutoCalibrationMode.cpp |
| Dima/rover/modes/auto_calibration/CalibrationParameters.hpp | 7 | Dima/rover/modes/auto_calibration/CalibrationParameters.cpp |

## 后续单行返回例外应用

2026-09-09 后续资源精简将 `param_handle()` 的单行 constexpr 转换恢复到头文件，以消除枚举转换的额外调用，符合当前用户约束。上面的 426 处迁移及 0 个普通运行期定义是该次冻结快照的历史统计，不作为禁止单行 return 的规则；后续变更与资源验收见 `CODE_SIZE_NEXT_OPTIMIZATIONS_ZH.md`。
