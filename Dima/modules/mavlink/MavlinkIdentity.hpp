#pragma once
/*
 * PX4 Rover identity for MAVLink.
 *
 * Provides the identity data needed for HEARTBEAT and AUTOPILOT_VERSION
 * messages: system/component IDs, vehicle type, autopilot type, firmware
 * version, capabilities bitmap, and hardware UID.
 *
 * This is a pure data provider — no thread, no work queue.
 */

#include <cstdint>
#include <cstring>

namespace dima::modules::mavlink {

/**
 * PX4 Rover identity constants and runtime state.
 *
 * The firmware version is encoded as a 32-bit value where each byte
 * represents (major)(minor)(patch)(FIRMWARE_VERSION_TYPE):
 *   type 0 = release, type 64 ('@') = alpha
 */
class MavlinkIdentity {
public:
    /* ── Fixed PX4 Rover identity ─────────────────────────────────── */

    static constexpr uint8_t  SYSTEM_ID   = 1;     /* PX4 autopilot default */
    static constexpr uint8_t  COMPONENT_ID = 1;    /* MAV_COMP_ID_AUTOPILOT1 */

    static constexpr uint8_t  MAV_TYPE_VALUE     = 10;   /* MAV_TYPE_GROUND_ROVER */
    static constexpr uint8_t  MAV_AUTOPILOT_VALUE = 12;  /* MAV_AUTOPILOT_PX4 */

    /* ── Firmware version ─────────────────────────────────────────── */

    /**
     * Encode semantic version into the PX4 flight_sw_version format.
     * Byte 3 (MSB) = major, byte 2 = minor, byte 1 = patch, byte 0 = type.
     */
    static constexpr uint32_t encode_version(uint8_t major, uint8_t minor,
                                              uint8_t patch, uint8_t type = 0)
    {
        return (static_cast<uint32_t>(major) << 24)
             | (static_cast<uint32_t>(minor) << 16)
             | (static_cast<uint32_t>(patch) << 8)
             |  static_cast<uint32_t>(type);
    }

    /* ── Capability flags ─────────────────────────────────────────── */

    static constexpr uint64_t CAPABILITY_PARAM_FLOAT     = 0x0002;
    static constexpr uint64_t CAPABILITY_MAVLINK2        = 0x0200;
    static constexpr uint64_t CAPABILITY_COMP_METADATA   = 0x2000;

    /* ── Constructor ──────────────────────────────────────────────── */

    MavlinkIdentity() = default;

    /**
     * Configure the identity with runtime values.
     *
     * @param version   Encoded firmware version (use encode_version()).
     * @param board_ver Board version from hardware.
     * @param uid       64-bit hardware UID (0 if unavailable).
     * @param vendor_id Board vendor ID (0 = unknown).
     * @param product_id Board product ID (0 = unknown).
     */
    void configure(uint32_t version, uint32_t board_version,
                   uint64_t uid, uint16_t vendor_id = 0,
                   uint16_t product_id = 0)
    {
        flight_sw_version_ = version;
        board_version_     = board_version;
        uid_               = uid;
        vendor_id_         = vendor_id;
        product_id_        = product_id;
    }

    /**
     * Set the current system state for HEARTBEAT.
     */
    void set_state(uint8_t base_mode, uint8_t system_status)
    {
        base_mode_    = base_mode;
        system_status_ = system_status;
    }

    /* ── Accessors ────────────────────────────────────────────────── */

    uint8_t  base_mode()      const noexcept { return base_mode_; }
    uint8_t  system_status()  const noexcept { return system_status_; }
    uint32_t flight_sw_version() const noexcept { return flight_sw_version_; }
    uint32_t board_version() const noexcept { return board_version_; }
    uint64_t uid()           const noexcept { return uid_; }
    uint16_t vendor_id()     const noexcept { return vendor_id_; }
    uint16_t product_id()    const noexcept { return product_id_; }

    uint64_t capabilities() const noexcept
    {
        return CAPABILITY_PARAM_FLOAT
             | CAPABILITY_MAVLINK2
             | CAPABILITY_COMP_METADATA;
    }

    /**
     * Get the git hash bytes (first 8 bytes) for custom version fields.
     * Currently returns zeros (no git hash embedded in this build).
     */
    void get_flight_custom_version(uint8_t out[8]) const noexcept
    {
        std::memset(out, 0, 8);
    }

    void get_middleware_custom_version(uint8_t out[8]) const noexcept
    {
        std::memset(out, 0, 8);
    }

    void get_os_custom_version(uint8_t out[8]) const noexcept
    {
        std::memset(out, 0, 8);
    }

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
