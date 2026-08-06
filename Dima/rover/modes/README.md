# Rover 产品模式

- **当前模式：** `ManualMode` 把 `manual_control_setpoint.throttle/yaw` 转换为 `rover_motion_request` 的前后、左右两轴；它是 Rover Manual 的运行入口，不是传输或 RC 适配器。
- **边界：** 模式不解析 RC、SBUS 或 MAVLink，不实现差速算法，不发布 `actuator_motors`，也不访问 MotorOutput/PWM。
- **扩展：** Navigation、Offboard 等后续模式直接作为本目录中的同级模式加入；模式较大时再建立同名子目录，禁止为单个源文件增加一层目录。
