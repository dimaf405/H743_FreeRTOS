#pragma once
/*
 * PX4 Rover identity for MAVLink.
 *
 * Provides the identity data needed for HEARTBEAT and AUTOPILOT_VERSION
 * messages: system/component IDs, vehicle type, autopilot type, firmware
 * version, capabilities bitmap, and hardware UID.
 *
 * This is a pure data provider — no thread, no work queue.
 *
 * 该对象只保存线协议身份与 HEARTBEAT 状态，不拥有线程、队列或硬件；所有
 * 运行时值均由 ApplicationContext/HeartbeatPacer 在明确时点写入。
 */

#include <cstdint>

namespace dima::modules::mavlink {

/**
 * PX4 Rover identity constants and runtime state.
 *
     * The generated firmware version follows MAVLink's 0xAABBCCTT layout, where
     * TT is FIRMWARE_VERSION_TYPE (0=development, 255=official release).
     * 其中 AA/BB/CC 分别是主/次/补丁版本，TT 是固件类型；编码值来自生成的
     * firmware identity 合同，不能在 MAVLink 模块内另建版本清单。
 */
class MavlinkIdentity {
public:
    /* 固定的 PX4 Rover 线协议身份；与 QGC 的 autopilot/vehicle 选择契约一致。 */

    static constexpr uint8_t  SYSTEM_ID   = 1;     /* PX4 autopilot default */
    static constexpr uint8_t  COMPONENT_ID = 1;    /* MAV_COMP_ID_AUTOPILOT1 */

    static constexpr uint8_t  MAV_TYPE_VALUE     = 10;   /* MAV_TYPE_GROUND_ROVER */
    static constexpr uint8_t  MAV_AUTOPILOT_VALUE = 12;  /* MAV_AUTOPILOT_PX4 */

    /* 构造后先保持零值，待板级 UID 和生成版本合同一起配置。 */

    MavlinkIdentity() = default;

    /**
     * Configure the identity with runtime values.
     * 同一次 configure 原子地形成 AUTOPILOT_VERSION 所需快照；调用方不得把
     * 未验证的上传包版本或主机推测值写入这里。
     *
     * @param version   Generated PX4/MAVLink encoded firmware version.
     * @param board_ver Board version from hardware.
     * @param uid       64-bit hardware UID (0 if unavailable).
     * @param vendor_id Board vendor ID (0 = unknown).
     * @param product_id Board product ID (0 = unknown).
     */
    void configure(uint32_t version, uint32_t board_version,
                   uint64_t uid, uint16_t vendor_id = 0,
                   uint16_t product_id = 0);

    /**
     * Set the current system state for HEARTBEAT.
     * base_mode/system_status 是当前安全与控制状态的投影，不是独立状态机。
     */
    void set_state(uint8_t base_mode, uint8_t system_status);

    /* 只读访问器供打包路径使用，避免发送线程重新推导身份或状态。 */

    uint8_t  base_mode()      const noexcept { return base_mode_; }
    uint8_t  system_status()  const noexcept { return system_status_; }
    uint32_t flight_sw_version() const noexcept { return flight_sw_version_; }
    uint32_t board_version() const noexcept { return board_version_; }
    uint64_t uid()           const noexcept { return uid_; }
    uint16_t vendor_id()     const noexcept { return vendor_id_; }
    uint16_t product_id()    const noexcept { return product_id_; }

    uint64_t capabilities() const noexcept;

    /**
     * Get the PX4 wire-order bytes for the first 16 Git hash characters.
     * 输入为生成合同中的 16 个十六进制字符，输出为 8 个真实字节；这是
     * MAVLink/PX4 wire order，不是把 ASCII 字符原样拷入消息。
     */
    void get_flight_custom_version(uint8_t out[8]) const noexcept;
    void get_middleware_custom_version(uint8_t out[8]) const noexcept;
    void get_os_custom_version(uint8_t out[8]) const noexcept;

private:
    uint32_t flight_sw_version_{0};
    uint32_t board_version_{0};
    uint64_t uid_{0};
    uint16_t vendor_id_{0};
    uint16_t product_id_{0};
    uint8_t  base_mode_{0};
    uint8_t  system_status_{0};
};

}  // namespace dima::modules::mavlink
