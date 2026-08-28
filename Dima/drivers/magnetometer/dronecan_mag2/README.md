# DroneCAN RM3100 磁力计链路与校准

本目录只拥有 RM3100/Mag2 transport 参数、source binding、Mag/Mag2 解码、500 ms timeout 处理与原始 `sensor_mag` 发布。
最小通用 DroneCAN node/session、NodeStatus 和 GetNodeInfo 由 `Dima/lib/dronecan/DroneCanNode.*` 负责，本驱动只消费该能力，不复制通用节点实现。

## 固定来源

- PX4-Autopilot v1.17.0：`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- DroneCAN DSDL：`993be80a62ec957c01fb41115b83663959a49f46`
- libcanard：`601ed35467e0ac38819df17cd7c918de19f62d58`
- vendored 来源记录：`Middlewares/Third_Party/libcanard/SOURCE.md` 与 `Middlewares/Third_Party/dronecan_dsdl/SOURCE.md`

## 运行合同

- `module_dronecan.yaml` 与其他 `module_*.yaml` 一样直接进入统一参数工具链；`UAVCAN1_ENABLE/BITRATE/NODE_ID` 位于 `UAVCAN` 参数组，`MAG1_CAN_NODE` 位于 `Magnetometer` 组。驱动通过生成的 `dima::ParamInt` 绑定这些参数，不保留 JSON、构建目录 YAML 或 DroneCAN 专用参数头。`SENS_MAG_RATE` 和 `CAL_MAG0_*` 只由独立 `VehicleMagnetometer` 前端拥有。
- 共享节点能力提供静态节点 ID、1 Hz NodeStatus 和 GetNodeInfo；本驱动订阅 Mag/Mag2，并由 transfer-ID tracker 拒绝重复/过期传输。
- DroneCAN 驱动只发布未套用 `CAL_MAG0_*` 的 `sensor_mag`，设备 ID 为 0、旧 ID 失配或校准无效都不能阻断原始数据。
- `Dima/modules/sensors/magnetometer/VehicleMagnetometer.*` 独立订阅 `sensor_mag`，按检测到的 device ID 选择匹配校准或 PX4 identity correction，再按 `SENS_MAG_RATE` 的 1..200 Hz 上限平均并发布 `vehicle_magnetometer`。该参数不改变远端 RM3100 的硬件采样率。
- 首个有效样本、恢复和 500 ms timeout 均产生日志。`HIGHRES_IMU` 提供校准后的实时磁场，`SYS_STATUS` 提供 MAG present/health。
- 未检测/超时日志同时报告 configured/active node、CAN RX、accepted/reject/duplicate/stale/decode/protocol、overrun、RX error、bus-off 和最后错误标志，可区分“总线没有任何帧”和“有帧但没有有效 Mag2 传输”。

## QGC 磁力计校准

`MAV_CMD_PREFLIGHT_CALIBRATION param2=1` 启动 Disarmed-only、PX4/QGC v2 六面校准。协调器运行在非实时 `wq:lp_default`，先用新鲜 accel/gyro 稳定识别 `back/front/left/right/up/down`，通过原样且不受普通日志等级过滤的 `[cal] <side> orientation detected` 驱动 QGC；随后要求陀螺仪有符号积分得到至少 0.5 rad 的实际净旋转。每面在 7 s 窗口内收集 40 个通过 PX4 空间间距去重的原始 `sensor_mag` 点，共固定 240 点，并用 `[cal] <side> side done, rotate to a different side` 完成界面状态转换。六面覆盖通过后使用固定内存最小二乘球拟合计算 hard-iron offset，并由三轴跨度求 diagonal scale；scale 接受范围与 PX4 `CAL_MAG0_*SCALE` 元数据及本地前端统一为 0.1..3.0。拟合范围、设备 ID 和样本新鲜度均通过后，原子写入 `CAL_MAG0_ID/XOFF/YOFF/ZOFF/XSCALE/YSCALE/ZSCALE`；只有独立 `VehicleMagnetometer` 前端确认同一 `parameter_update.instance`、逐项匹配 active correction，并发布新的 calibration count 后才报告完成。取消或失败后的参数回滚使用同一握手，完成前保持 arming interlock。

当前产品只支持 diagonal scale，不声称实现 PX4 完整的非对角 soft-iron 拟合。FDCAN 位时序、收发器 silent 控制、RM3100 节点绑定、热插拔、实际磁场方向、带载干扰与 QGC 实物校准仍为 `BOARD PENDING`。
