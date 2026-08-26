#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::protocols::um982 {

// 无动态分配的增量式 UM982 解析器。它同时接受 '$' 开头的 NMEA/配置响应
// 和 '#' 开头的 Unicore 二进制风格 ASCII 日志，并把校验错误、结构错误、
// 超长帧和未知但合法消息分开，供驱动采用不同失败语义。
class Um982Protocol final {
public:
    // 4096 B 是单帧硬上限（另留一个 NUL）；跟踪的日志数由生成合同限制为 8，
    // 接收机物理 COM 端口固定为 3。所有容量均编译期确定。
    static constexpr std::size_t kMaximumFrameBytes = 4096U;
    static constexpr std::uint8_t kMaximumTrackedLogs = 8U;
    static constexpr std::uint8_t kReceiverPortCount = 3U;

    enum class Kind : std::uint8_t {
        None,
        Version,
        ConfigPort,
        UnilogList,
        Gga,
        Gst,
        Gsa,
        Rmc,
        Agrica,
        Heading,
        Unknown,
        BadChecksum,
        BadStructure,
        Overflow,
    };

    struct Version {
        bool is_um982{false};
    };

    struct ConfigPort {
        std::uint8_t port{0U};
        std::uint32_t baudrate{0U};
    };

    struct UnilogList {
        std::uint8_t present_mask[kReceiverPortCount]{};
        std::uint8_t instance_count[kReceiverPortCount][kMaximumTrackedLogs]{};
        float period_s[kReceiverPortCount][kMaximumTrackedLogs]{};
    };

    struct Gga {
        // 经纬度单位为度，高度/大地水准面差单位为米，UTC 保存为
        // hhmmss*1000+毫秒；fix_quality 沿用 NMEA GGA 质量编码。
        double latitude_deg{0.0};
        double longitude_deg{0.0};
        float altitude_msl_m{0.0F};
        float geoid_separation_m{0.0F};
        float hdop{0.0F};
        std::uint32_t utc_hhmmss_ms{0U};
        std::uint8_t fix_quality{0U};
        std::uint8_t satellites{0U};
    };

    struct Gst {
        // 位置 RMS/标准差单位均为米。
        float rms_m{0.0F};
        float latitude_stddev_m{0.0F};
        float longitude_stddev_m{0.0F};
        float altitude_stddev_m{0.0F};
    };

    struct Gsa {
        float pdop{0.0F};
        float hdop{0.0F};
        float vdop{0.0F};
        std::uint8_t fix_dimension{0U};
    };

    struct Rmc {
        // 地速在解析时由节换算为 m/s，course 保持真北顺时针角度制。
        double latitude_deg{0.0};
        double longitude_deg{0.0};
        float ground_speed_m_s{0.0F};
        float course_deg{0.0F};
        std::uint32_t utc_hhmmss_ms{0U};
        std::uint32_t date_ddmmyy{0U};
        bool valid{false};
    };

    struct Agrica {
        // 速度为本地 N/E/U（m/s），精度字段为各轴 1-sigma（m/s）。
        float speed_m_s{0.0F};
        float velocity_north_m_s{0.0F};
        float velocity_east_m_s{0.0F};
        float velocity_up_m_s{0.0F};
        float velocity_north_stddev_m_s{0.0F};
        float velocity_east_stddev_m_s{0.0F};
        float velocity_up_stddev_m_s{0.0F};
        std::uint32_t gps_milliseconds{0U};
        std::uint16_t gps_week{0U};
        std::uint8_t position_type{0U};
        std::uint8_t heading_status{0U};
    };

    struct Heading {
        // 双天线基线单位 m，航向及 1-sigma 精度单位 deg；无解算时数值为 NaN。
        float baseline_m{0.0F};
        float heading_deg{0.0F};
        float heading_stddev_deg{0.0F};
        std::uint32_t gps_milliseconds{0U};
        std::uint16_t gps_week{0U};
        bool solution_computed{false};
    };

    struct Frame {
        Kind kind{Kind::None};
        Version version{};
        ConfigPort config_port{};
        UnilogList unilog_list{};
        Gga gga{};
        Gst gst{};
        Gsa gsa{};
        Rmc rmc{};
        Agrica agrica{};
        Heading heading{};
    };

    // 每输入一个字节推进状态机；仅在完整帧或明确错误完成时返回 true。
    bool feed(std::uint8_t byte, Frame &frame) noexcept;
    void reset() noexcept;

    // 命令线只追加 CRLF；NMEA 校验为 '*' 前正文逐字节 XOR，Unicore 使用
    // 反射多项式 0xEDB88320 的 CRC-32。utc_usec 返回 Unix epoch 微秒。
    static std::size_t make_command(const char *body, char *destination,
                                    std::size_t capacity) noexcept;
    static std::uint8_t nmea_xor(const char *data,
                                 std::size_t length) noexcept;
    static std::uint32_t unicore_crc32(const char *data,
                                       std::size_t length) noexcept;
    static std::uint64_t utc_usec(std::uint32_t date_ddmmyy,
                                  std::uint32_t time_hhmmss_ms) noexcept;

private:
    bool complete(Frame &frame) noexcept;

    // checksum_digits_ 从看到 '*' 起以 1 计数；NMEA 总共需要 2 个十六进制位，
    // Unicore 需要 8 个。start_=='\0' 表示当前没有正在组装的帧。
    char buffer_[kMaximumFrameBytes + 1U]{};
    std::size_t length_{0U};
    std::size_t checksum_offset_{0U};
    std::uint8_t checksum_digits_{0U};
    std::uint8_t checksum_digits_required_{0U};
    char start_{'\0'};
};

} // namespace dima::protocols::um982
