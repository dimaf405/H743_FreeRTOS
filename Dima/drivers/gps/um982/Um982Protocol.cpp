#include "Um982Protocol.hpp"
#include "Um982MessageContract.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace dima::protocols::um982 {
namespace {

bool hex_digit(char value) noexcept
{
    return (value >= '0' && value <= '9') ||
           (value >= 'a' && value <= 'f') ||
           (value >= 'A' && value <= 'F');
}

std::uint32_t parse_hex(const char *text, std::size_t length) noexcept
{
    // 十六进制逐 nibble 左移拼接；调用者已逐字符验证，因此此处不接受前缀、
    // 空白或符号，避免宽松 libc 转换吞掉损坏的校验字段。
    std::uint32_t value = 0U;
    for (std::size_t index = 0U; index < length; ++index) {
        const char digit = text[index];
        const std::uint32_t nibble =
            digit >= '0' && digit <= '9'
                ? static_cast<std::uint32_t>(digit - '0')
                : (digit >= 'a' && digit <= 'f'
                       ? static_cast<std::uint32_t>(digit - 'a' + 10)
                       : static_cast<std::uint32_t>(digit - 'A' + 10));
        value = (value << 4U) | nibble;
    }
    return value;
}

std::size_t split(char *text, char delimiter, char **fields,
                  std::size_t capacity) noexcept
{
    // 原地把分隔符替换为 NUL，并最多返回 capacity 个视图；不分配内存。
    // 超出容量的尾部仍被切开但不返回，随后各消息解析器会因字段数不足而拒绝。
    if (text == nullptr || fields == nullptr || capacity == 0U) {
        return 0U;
    }
    std::size_t count = 0U;
    fields[count++] = text;
    for (char *cursor = text; *cursor != '\0'; ++cursor) {
        if (*cursor == delimiter) {
            *cursor = '\0';
            if (count < capacity) {
                fields[count++] = cursor + 1;
            }
        }
    }
    return count;
}

bool parse_unsigned_prefix(const char *text, const char *&end,
                           std::uint32_t &value) noexcept
{
    if (text == nullptr || *text == '\0') return false;
    std::uint32_t parsed = 0U;
    const char *cursor = text;
    bool have_digit = false;
    while (*cursor >= '0' && *cursor <= '9') {
        const std::uint32_t digit = static_cast<std::uint32_t>(
            *cursor - '0');
        // 在 parsed*10+digit 前验证 parsed <= (UINT32_MAX-digit)/10。
        if (parsed > (UINT32_MAX - digit) / 10U) return false;
        parsed = parsed * 10U + digit;
        have_digit = true;
        ++cursor;
    }
    if (!have_digit) return false;
    end = cursor;
    value = parsed;
    return true;
}

bool parse_decimal_prefix(const char *text, const char *&end,
                           double &value) noexcept
{
    // 固定语法的十进制解析避免 strtod 的 locale、errno 和动态运行库依赖；
    // 接受可选符号、小数与 |exponent|<=308，最终必须为有限值。
    if (text == nullptr || *text == '\0') return false;
    const char *cursor = text;
    bool negative = false;
    if (*cursor == '+' || *cursor == '-') {
        negative = *cursor == '-';
        ++cursor;
    }

    double parsed = 0.0;
    bool have_digit = false;
    while (*cursor >= '0' && *cursor <= '9') {
        if (parsed > std::numeric_limits<double>::max() / 10.0) return false;
        parsed = parsed * 10.0 + static_cast<double>(*cursor - '0');
        have_digit = true;
        ++cursor;
    }
    if (*cursor == '.') {
        ++cursor;
        double place = 0.1;
        while (*cursor >= '0' && *cursor <= '9') {
            parsed += static_cast<double>(*cursor - '0') * place;
            place *= 0.1;
            have_digit = true;
            ++cursor;
        }
    }
    if (!have_digit) return false;

    int exponent = 0;
    if (*cursor == 'e' || *cursor == 'E') {
        ++cursor;
        bool exponent_negative = false;
        if (*cursor == '+' || *cursor == '-') {
            exponent_negative = *cursor == '-';
            ++cursor;
        }
        std::uint32_t exponent_magnitude = 0U;
        const char *exponent_end = nullptr;
        if (!parse_unsigned_prefix(cursor, exponent_end,
                                   exponent_magnitude) ||
            exponent_magnitude > 308U) {
            return false;
        }
        cursor = exponent_end;
        exponent = exponent_negative
                       ? -static_cast<int>(exponent_magnitude)
                       : static_cast<int>(exponent_magnitude);
    }
    if (exponent > 0) {
        while (exponent-- > 0) parsed *= 10.0;
    } else {
        while (exponent++ < 0) parsed *= 0.1;
    }
    if (negative) parsed = -parsed;
    if (!std::isfinite(parsed)) return false;
    end = cursor;
    value = parsed;
    return true;
}

bool parse_float(const char *text, float &value) noexcept
{
    const char *end = nullptr;
    double parsed = 0.0;
    if (!parse_decimal_prefix(text, end, parsed) || *end != '\0') return false;
    const float converted = static_cast<float>(parsed);
    if (!std::isfinite(converted)) return false;
    value = converted;
    return true;
}

bool parse_double(const char *text, double &value) noexcept
{
    const char *end = nullptr;
    return parse_decimal_prefix(text, end, value) && *end == '\0';
}

bool parse_unsigned(const char *text, std::uint32_t &value) noexcept
{
    const char *end = nullptr;
    return parse_unsigned_prefix(text, end, value) && *end == '\0';
}

bool parse_optional_nonnegative_float(const char *text,
                                      float &value) noexcept
{
    if (text == nullptr || *text == '\0') {
        value = std::numeric_limits<float>::quiet_NaN();
        return true;
    }
    return parse_float(text, value) && value >= 0.0F;
}

bool parse_px4_gst_float(const char *text, float &value) noexcept
{
    /* PX4-GPSDrivers nmea.cpp:621-679 keeps the zero-initialized GST value
     * when UM982 emits an empty field. A non-empty field must still be a
     * complete decimal in this bounded parser. */
    if (text == nullptr || *text == '\0') {
        value = 0.0F;
        return true;
    }
    return parse_float(text, value);
}

double nmea_coordinate(const char *text, const char *hemisphere,
                       bool latitude, bool &valid) noexcept
{
    double raw = 0.0;
    const bool hemisphere_valid = hemisphere != nullptr &&
        hemisphere[0] != '\0' && hemisphere[1] == '\0' &&
        (latitude ? (hemisphere[0] == 'N' || hemisphere[0] == 'S')
                  : (hemisphere[0] == 'E' || hemisphere[0] == 'W'));
    valid = parse_double(text, raw) && raw >= 0.0 && hemisphere_valid;
    if (!valid) return 0.0;
    // NMEA 坐标为 ddmm.mmmm/dddmm.mmmm：deg=floor(raw/100)，
    // decimal_deg=deg+(raw-deg*100)/60；S/W 半球取负。
    const double degrees = std::floor(raw / 100.0);
    const double minutes = raw - degrees * 100.0;
    const double maximum_degrees = latitude ? 90.0 : 180.0;
    if (minutes < 0.0 || minutes >= 60.0 ||
        degrees > maximum_degrees ||
        (degrees == maximum_degrees && minutes > 0.0)) {
        valid = false;
        return 0.0;
    }
    double converted = degrees + minutes / 60.0;
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') converted = -converted;
    return converted;
}

bool parse_hhmmss_ms(const char *text, std::uint32_t &result) noexcept
{
    // 输入 hhmmss.sss，输出紧凑整数 hhmmss*1000+ms；小数按最近毫秒取整，
    // 999 ms 封顶，避免浮点舍入产生非法的“第 1000 毫秒”。
    result = 0U;
    double raw = 0.0;
    if (!parse_double(text, raw) || raw < 0.0 || raw >= 240000.0) {
        return false;
    }
    const std::uint32_t whole = static_cast<std::uint32_t>(raw);
    const std::uint32_t hours = whole / 10000U;
    const std::uint32_t minutes = (whole / 100U) % 100U;
    const std::uint32_t seconds = whole % 100U;
    if (hours >= 24U || minutes >= 60U || seconds >= 60U) return false;
    const double fraction = raw - static_cast<double>(whole);
    const std::uint32_t millis = static_cast<std::uint32_t>(
        fraction * 1000.0 + 0.5);
    result = ((hours * 10000U + minutes * 100U + seconds) * 1000U) +
             (millis > 999U ? 999U : millis);
    return true;
}

bool parse_unicore_header(char *header, std::uint16_t &week,
                           std::uint32_t &milliseconds) noexcept
{
    // Unicore ASCII 头第 5/6 字段分别为 GPS week 与周内毫秒；周内值必须
    // 小于 7*24*3600*1000=604800000。
    char *fields[12]{};
    const std::size_t count = split(header, ',', fields, 12U);
    std::uint32_t parsed_week = 0U;
    return count >= 6U && parse_unsigned(fields[4], parsed_week) &&
           parsed_week <= UINT16_MAX &&
           parse_unsigned(fields[5], milliseconds) &&
           milliseconds < 604800000U
                ? (week = static_cast<std::uint16_t>(parsed_week), true)
                : false;
}

bool parse_agrica(char *header, char *data,
                  Um982Protocol::Frame &frame) noexcept
{
    // AGRICA 字段位置是 UM982 协议合同：位置/航向状态在 8/9，速度 N/E/U
    // 在 22..24（索引 21..23），其 1-sigma 在随后三项；单位均为 m/s。
    if (!parse_unicore_header(header, frame.agrica.gps_week,
                              frame.agrica.gps_milliseconds)) return false;
    char *fields[72]{};
    const std::size_t count = split(data, ',', fields, 72U);
    std::uint32_t position_type = 0U;
    std::uint32_t heading_status = 0U;
    if (count <= 27U || !parse_unsigned(fields[8], position_type) ||
        !parse_unsigned(fields[9], heading_status) ||
        position_type > UINT8_MAX || heading_status > UINT8_MAX ||
        !parse_float(fields[21], frame.agrica.speed_m_s) ||
        !parse_float(fields[22], frame.agrica.velocity_north_m_s) ||
        !parse_float(fields[23], frame.agrica.velocity_east_m_s) ||
        !parse_float(fields[24], frame.agrica.velocity_up_m_s) ||
        !parse_float(fields[25], frame.agrica.velocity_north_stddev_m_s) ||
        !parse_float(fields[26], frame.agrica.velocity_east_stddev_m_s) ||
        !parse_float(fields[27], frame.agrica.velocity_up_stddev_m_s) ||
        frame.agrica.speed_m_s < 0.0F ||
        frame.agrica.velocity_north_stddev_m_s < 0.0F ||
        frame.agrica.velocity_east_stddev_m_s < 0.0F ||
        frame.agrica.velocity_up_stddev_m_s < 0.0F) {
        return false;
    }
    frame.agrica.position_type = static_cast<std::uint8_t>(position_type);
    frame.agrica.heading_status = static_cast<std::uint8_t>(heading_status);
    return true;
}

bool parse_heading(char *header, char *data,
                    Um982Protocol::Frame &frame) noexcept
{
    if (!parse_unicore_header(header, frame.heading.gps_week,
                              frame.heading.gps_milliseconds)) return false;
    char *fields[24]{};
    const std::size_t count = split(data, ',', fields, 24U);
    if (count < 7U ||
        !parse_float(fields[2], frame.heading.baseline_m) ||
        !parse_float(fields[3], frame.heading.heading_deg) ||
        !parse_float(fields[6], frame.heading.heading_stddev_deg)) {
        return false;
    }
    // 仅 SOL_COMPUTED 的正基线、[0,360] deg 航向和正精度可参与融合；其他
    // 合法状态仍作为在线帧接收，但数值显式改为 NaN，防止旧值被误用。
    frame.heading.solution_computed =
        std::strcmp(fields[0], "SOL_COMPUTED") == 0;
    if (frame.heading.solution_computed) {
        if (frame.heading.baseline_m <= 0.0F ||
            frame.heading.heading_deg < 0.0F ||
            frame.heading.heading_deg > 360.0F ||
            frame.heading.heading_stddev_deg <= 0.0F ||
            frame.heading.heading_stddev_deg > 360.0F) {
            return false;
        }
    } else {
        const float unavailable = std::numeric_limits<float>::quiet_NaN();
        frame.heading.baseline_m = unavailable;
        frame.heading.heading_deg = unavailable;
        frame.heading.heading_stddev_deg = unavailable;
    }
    return true;
}

bool parse_version(char *data, Um982Protocol::Frame &frame) noexcept
{
    if (data == nullptr) return false;
    frame.version.is_um982 = std::strstr(data, "UM982") != nullptr;
    return frame.version.is_um982;
}

bool parse_config(char *body, Um982Protocol::Frame &frame) noexcept
{
    // CONFIG 回读包含外层响应字段和内层命令文本，两层都必须严格匹配，
    // 最终只接受 COM1..COM3，避免任意相似字符串污染端口识别。
    char *fields[4]{};
    const std::size_t count = split(body, ',', fields, 4U);
    if (count < 3U || std::strcmp(fields[0], "CONFIG") != 0 ||
        std::strncmp(fields[1], "COM", 3U) != 0) {
        return false;
    }

    char *tokens[6]{};
    const std::size_t token_count = split(fields[2], ' ', tokens, 6U);
    if (token_count < 3U || std::strcmp(tokens[0], "CONFIG") != 0 ||
        std::strncmp(tokens[1], "COM", 3U) != 0) {
        return false;
    }

    std::uint32_t port = 0U;
    std::uint32_t baud = 0U;
    if (!parse_unsigned(tokens[1] + 3U, port) ||
        !parse_unsigned(tokens[2], baud) || port < 1U ||
        port > Um982Protocol::kReceiverPortCount) {
        return false;
    }

    frame.config_port.port = static_cast<std::uint8_t>(port);
    frame.config_port.baudrate = baud;
    return true;
}

bool parse_gga(char **fields, std::size_t count,
               Um982Protocol::Frame &frame) noexcept
{
    if (count < 13U) return false;
    std::uint32_t quality = 0U;
    std::uint32_t satellites = 0U;
    if (!parse_unsigned(fields[6], quality) || quality > 8U ||
        !parse_unsigned(fields[7], satellites) ||
        satellites > UINT8_MAX) {
        return false;
    }
    frame.gga.fix_quality = static_cast<std::uint8_t>(quality);
    frame.gga.satellites = static_cast<std::uint8_t>(satellites);

    /* A standards-compliant no-fix GGA normally leaves position and
     * altitude fields empty. It still proves that the receiver and UART are
     * alive, so preserve it as a valid NO_FIX sample without inventing a
     * position. A frame that claims a fix remains subject to the strict
     * coordinate and altitude validation below. */
    // NO_FIX GGA 仍证明 UART/接收机在线，但不提供位置。空字段转换为 NaN；若
    // 接收机声称有 fix，则继续执行严格经纬度、高度、HDOP 和卫星数检查。
    if (quality == 0U) {
        if (fields[1][0] != '\0' &&
            !parse_hhmmss_ms(fields[1], frame.gga.utc_hhmmss_ms)) {
            return false;
        }
        const bool position_supplied = fields[2][0] != '\0' ||
            fields[3][0] != '\0' || fields[4][0] != '\0' ||
            fields[5][0] != '\0';
        if (position_supplied) {
            bool lat_valid = false;
            bool lon_valid = false;
            (void)nmea_coordinate(fields[2], fields[3], true, lat_valid);
            (void)nmea_coordinate(fields[4], fields[5], false, lon_valid);
            if (!lat_valid || !lon_valid) {
                return false;
            }
        }
        float discarded = 0.0F;
        if ((fields[9][0] != '\0' &&
             !parse_float(fields[9], discarded)) ||
            (fields[11][0] != '\0' &&
             !parse_float(fields[11], discarded))) {
            return false;
        }
        const float unavailable = std::numeric_limits<float>::quiet_NaN();
        frame.gga.latitude_deg =
            std::numeric_limits<double>::quiet_NaN();
        frame.gga.longitude_deg =
            std::numeric_limits<double>::quiet_NaN();
        frame.gga.altitude_msl_m = unavailable;
        frame.gga.geoid_separation_m = unavailable;
        frame.gga.hdop = unavailable;
        if (fields[8][0] != '\0' &&
            (!parse_float(fields[8], frame.gga.hdop) ||
             frame.gga.hdop < 0.0F)) {
            return false;
        }
        return true;
    }

    bool lat_valid = false;
    bool lon_valid = false;
    const bool utc_valid = parse_hhmmss_ms(fields[1], frame.gga.utc_hhmmss_ms);
    frame.gga.latitude_deg = nmea_coordinate(
        fields[2], fields[3], true, lat_valid);
    frame.gga.longitude_deg = nmea_coordinate(
        fields[4], fields[5], false, lon_valid);
    return lat_valid && lon_valid && utc_valid &&
           parse_float(fields[8], frame.gga.hdop) &&
           parse_float(fields[9], frame.gga.altitude_msl_m) &&
           parse_float(fields[11], frame.gga.geoid_separation_m) &&
           frame.gga.hdop >= 0.0F && satellites > 0U;
}

bool parse_gst(char **fields, std::size_t count,
               Um982Protocol::Frame &frame) noexcept
{
    return count == 9U &&
           parse_px4_gst_float(fields[2], frame.gst.rms_m) &&
           parse_px4_gst_float(fields[6], frame.gst.latitude_stddev_m) &&
           parse_px4_gst_float(fields[7], frame.gst.longitude_stddev_m) &&
           parse_px4_gst_float(fields[8], frame.gst.altitude_stddev_m);
}

bool parse_gsa(char **fields, std::size_t count,
               Um982Protocol::Frame &frame) noexcept
{
    if (count < 18U) return false;
    std::uint32_t dimension = 0U;
    const std::size_t dop = count >= 19U ? count - 4U : 15U;
    return dop + 2U < count && parse_unsigned(fields[2], dimension) &&
           dimension >= 1U && dimension <= 3U &&
           parse_optional_nonnegative_float(fields[dop], frame.gsa.pdop) &&
           parse_optional_nonnegative_float(fields[dop + 1U],
                                            frame.gsa.hdop) &&
           parse_optional_nonnegative_float(fields[dop + 2U],
                                            frame.gsa.vdop)
               ? (frame.gsa.fix_dimension =
                      static_cast<std::uint8_t>(dimension),
                  true)
               : false;
}

bool parse_rmc(char **fields, std::size_t count,
               Um982Protocol::Frame &frame) noexcept
{
    if (count < 10U) return false;
    const bool active = std::strcmp(fields[2], "A") == 0;
    const bool invalid = std::strcmp(fields[2], "V") == 0;
    if (!active && !invalid) return false;
    frame.rmc.valid = active;

    /* Status V is the normal pre-fix state. Empty position, speed and course
     * must not hide an otherwise live UM982 from receiver validation and
     * GPS_RAW_INT(NO_FIX). */
    // RMC 状态 V 是正常的未定位阶段：允许关键测量为空并输出 NaN，仍保留在线
    // 证据；状态 A 则要求完整 UTC、日期、位置、非负地速和合法航迹。
    if (invalid) {
        if (fields[1][0] != '\0' &&
            !parse_hhmmss_ms(fields[1], frame.rmc.utc_hhmmss_ms)) {
            return false;
        }
        if (fields[9][0] != '\0' &&
            !parse_unsigned(fields[9], frame.rmc.date_ddmmyy)) {
            return false;
        }
        const bool position_supplied = fields[3][0] != '\0' ||
            fields[4][0] != '\0' || fields[5][0] != '\0' ||
            fields[6][0] != '\0';
        if (position_supplied) {
            bool lat_valid = false;
            bool lon_valid = false;
            (void)nmea_coordinate(fields[3], fields[4], true, lat_valid);
            (void)nmea_coordinate(fields[5], fields[6], false, lon_valid);
            if (!lat_valid || !lon_valid) {
                return false;
            }
        }
        float discarded = 0.0F;
        if (fields[7][0] != '\0' &&
            (!parse_float(fields[7], discarded) || discarded < 0.0F)) {
            return false;
        }
        if (fields[8][0] != '\0' &&
            (!parse_float(fields[8], discarded) || discarded < 0.0F ||
             discarded > 360.0F)) {
            return false;
        }
        if (fields[1][0] != '\0' && fields[9][0] != '\0' &&
            Um982Protocol::utc_usec(frame.rmc.date_ddmmyy,
                                   frame.rmc.utc_hhmmss_ms) == 0U) {
            return false;
        }
        frame.rmc.latitude_deg =
            std::numeric_limits<double>::quiet_NaN();
        frame.rmc.longitude_deg =
            std::numeric_limits<double>::quiet_NaN();
        frame.rmc.ground_speed_m_s =
            std::numeric_limits<float>::quiet_NaN();
        frame.rmc.course_deg = std::numeric_limits<float>::quiet_NaN();
        return true;
    }

    bool lat_valid = false;
    bool lon_valid = false;
    const bool utc_valid = parse_hhmmss_ms(fields[1], frame.rmc.utc_hhmmss_ms);
    frame.rmc.latitude_deg = nmea_coordinate(
        fields[3], fields[4], true, lat_valid);
    frame.rmc.longitude_deg = nmea_coordinate(
        fields[5], fields[6], false, lon_valid);
    std::uint32_t date = 0U;
    float knots = 0.0F;
    if (!lat_valid || !lon_valid || !utc_valid ||
        !parse_float(fields[7], knots) ||
        !parse_float(fields[8], frame.rmc.course_deg) ||
        !parse_unsigned(fields[9], date)) return false;
    if (knots < 0.0F || frame.rmc.course_deg < 0.0F ||
        frame.rmc.course_deg > 360.0F ||
        Um982Protocol::utc_usec(date, frame.rmc.utc_hhmmss_ms) == 0U) {
        return false;
    }
    // 1 knot = 1852/3600 m/s = 0.514444444... m/s。
    frame.rmc.ground_speed_m_s = knots * 0.514444444F;
    frame.rmc.date_ddmmyy = date;
    return true;
}

bool parse_unilog(char *body, Um982Protocol::Frame &frame) noexcept
{
    // 只遍历生成合同中的日志名，并按 COM 分别统计存在位、实例数与周期；
    // instance_count 饱和到 UINT8_MAX，使重复实例不会因回绕伪装成单实例。
    for (std::size_t index = 0U;
         index < generated::kMessageContractCount; ++index) {
        char *search = body;
        const char *const log_name =
            generated::kMessageContracts[index].log_name;
        const std::size_t name_length = std::strlen(log_name);
        while (char *const entry = std::strstr(search,
                                               log_name)) {
            search = entry + name_length;
            const char *cursor = search;
            while (*cursor == ' ') ++cursor;
            if (std::strncmp(cursor, "COM", 3U) != 0) continue;
            cursor += 3U;

            const char *end = nullptr;
            std::uint32_t port = 0U;
            if (!parse_unsigned_prefix(cursor, end, port) || port < 1U ||
                port > Um982Protocol::kReceiverPortCount) {
                continue;
            }
            cursor = end;
            while (*cursor == ' ') ++cursor;
            double parsed_period = 0.0;
            if (!parse_decimal_prefix(cursor, end, parsed_period)) continue;
            const float period = static_cast<float>(parsed_period);
            if (!std::isfinite(period) || period <= 0.0F) continue;

            const std::uint8_t port_index =
                static_cast<std::uint8_t>(port - 1U);
            frame.unilog_list.present_mask[port_index] |=
                static_cast<std::uint8_t>(1U << index);
            std::uint8_t &instance_count =
                frame.unilog_list.instance_count[port_index][index];
            if (instance_count != UINT8_MAX) ++instance_count;
            frame.unilog_list.period_s[port_index][index] = period;
        }
    }
    return true;
}

bool leap_year(int year) noexcept
{
    return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

std::int64_t days_from_civil(int year, unsigned month, unsigned day) noexcept
{
    // 公历日期按 400 年 era 分解，返回相对 1970-01-01 的天数；常量 719468
    // 是民用纪元到 Unix epoch 的偏移。该整数算法不依赖本地时区或 libc。
    year -= month <= 2U;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(year - era * 400);
    const unsigned adjusted_month = month > 2U ? month - 3U : month + 9U;
    const unsigned doy = (153U * adjusted_month + 2U) / 5U +
                         day - 1U;
    const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return static_cast<std::int64_t>(era) * 146097LL +
           static_cast<std::int64_t>(doe) - 719468LL;
}

} // namespace

bool Um982Protocol::feed(std::uint8_t byte, Frame &frame) noexcept
{
    frame = Frame{};
    const char value = static_cast<char>(byte);
    // 空闲态只接受 '$' 或 '#'；其他线路噪声直接忽略，不产生逐字节错误事件。
    if (start_ == '\0') {
        if (value != '$' && value != '#') return false;
        start_ = value;
        checksum_digits_required_ = value == '$' ? 2U : 8U;
        buffer_[0] = value;
        length_ = 1U;
        return false;
    }

    // 校验段开始前出现新的起始符，说明上一帧截断：直接以新帧重新同步。
    if ((value == '$' || value == '#') && checksum_digits_ == 0U) {
        reset();
        start_ = value;
        checksum_digits_required_ = value == '$' ? 2U : 8U;
        buffer_[0] = value;
        length_ = 1U;
        return false;
    }

    // 写入前检查 4096 B 硬上限，另一个字节始终保留给 NUL 终止符。
    if (length_ >= kMaximumFrameBytes) {
        reset();
        frame.kind = Kind::Overflow;
        return true;
    }
    buffer_[length_++] = value;
    buffer_[length_] = '\0';

    if (checksum_digits_ == 0U) {
        if (value == '*') {
            checksum_offset_ = length_ - 1U;
            checksum_digits_ = 1U;
        }
        return false;
    }

    if (!hex_digit(value)) {
        reset();
        frame.kind = Kind::BadStructure;
        return true;
    }
    if (checksum_digits_ < checksum_digits_required_) {
        ++checksum_digits_;
        return false;
    }
    return complete(frame);
}

void Um982Protocol::reset() noexcept
{
    length_ = 0U;
    checksum_offset_ = 0U;
    checksum_digits_ = 0U;
    checksum_digits_required_ = 0U;
    start_ = '\0';
    buffer_[0] = '\0';
}

bool Um982Protocol::complete(Frame &frame) noexcept
{
    const char start = start_;
    const std::size_t checksum_length = checksum_digits_required_;
    const std::size_t checksum_offset = checksum_offset_;
    const std::uint32_t supplied = parse_hex(
        &buffer_[checksum_offset + 1U], checksum_length);
    // 校验范围都排除起始符与 '*'：NMEA 为正文 XOR，Unicore 为正文 CRC-32。
    const std::uint32_t calculated = start == '$'
        ? nmea_xor(&buffer_[1], checksum_offset - 1U)
        : unicore_crc32(&buffer_[1], checksum_offset - 1U);
    if (supplied != calculated) {
        reset();
        frame.kind = Kind::BadChecksum;
        return true;
    }

    buffer_[checksum_offset] = '\0';
    char *body = &buffer_[1];
    bool parsed = false;
    // 校验正确后再原地分割。未知但语法/校验合法的消息返回 Unknown；只有已知
    // 消息字段不满足合同才返回 BadStructure，便于上层采用不同健康语义。
    if (start == '$') {
        if (std::strncmp(body, "CONFIG,", 7U) == 0) {
            parsed = parse_config(body, frame);
            frame.kind = parsed ? Kind::ConfigPort : Kind::BadStructure;
        } else {
            char *fields[24]{};
            const std::size_t count = split(body, ',', fields, 24U);
            const char *name = count > 0U ? fields[0] : "";
            const std::size_t name_length = std::strlen(name);
            const char *suffix = name_length >= 3U
                                     ? name + name_length - 3U
                                     : name;
            if (std::strcmp(suffix, "GGA") == 0) {
                parsed = parse_gga(fields, count, frame);
                frame.kind = parsed ? Kind::Gga : Kind::BadStructure;
            } else if (std::strcmp(suffix, "GST") == 0) {
                parsed = parse_gst(fields, count, frame);
                frame.kind = parsed ? Kind::Gst : Kind::BadStructure;
            } else if (std::strcmp(suffix, "GSA") == 0) {
                parsed = parse_gsa(fields, count, frame);
                frame.kind = parsed ? Kind::Gsa : Kind::BadStructure;
            } else if (std::strcmp(suffix, "RMC") == 0) {
                parsed = parse_rmc(fields, count, frame);
                frame.kind = parsed ? Kind::Rmc : Kind::BadStructure;
            } else {
                frame.kind = Kind::Unknown;
            }
        }
    } else {
        char *semicolon = std::strchr(body, ';');
        if (semicolon != nullptr) {
            *semicolon = '\0';
            char message[24]{};
            const char *comma = std::strchr(body, ',');
            const std::size_t message_length = comma == nullptr
                ? std::strlen(body)
                : static_cast<std::size_t>(comma - body);
            const std::size_t copied = message_length < sizeof(message) - 1U
                                           ? message_length
                                           : sizeof(message) - 1U;
            std::memcpy(message, body, copied);
            char *data = semicolon + 1;
            if (std::strcmp(message, "VERSIONA") == 0) {
                parsed = parse_version(data, frame);
                frame.kind = parsed ? Kind::Version : Kind::BadStructure;
            } else if (std::strcmp(message, "AGRICA") == 0 ||
                       std::strcmp(message, "UNIAGRICA") == 0) {
                parsed = parse_agrica(body, data, frame);
                frame.kind = parsed ? Kind::Agrica : Kind::BadStructure;
            } else if (std::strcmp(message, "UNIHEADINGA") == 0) {
                parsed = parse_heading(body, data, frame);
                frame.kind = parsed ? Kind::Heading : Kind::BadStructure;
            } else if (std::strcmp(message, "UNILOGLIST") == 0) {
                parsed = parse_unilog(data, frame);
                frame.kind = parsed ? Kind::UnilogList : Kind::BadStructure;
            } else {
                frame.kind = Kind::Unknown;
            }
        } else {
            frame.kind = Kind::Unknown;
        }
    }
    reset();
    return true;
}

std::size_t Um982Protocol::make_command(const char *body, char *destination,
                                         std::size_t capacity) noexcept
{
    // required=正文长度+CRLF；额外再保留 1 B NUL，仅用于本地安全调试，
    // 实际 UART 写入长度不包含该 NUL。
    if (body == nullptr || destination == nullptr || capacity == 0U) return 0U;
    const std::size_t length = std::strlen(body);
    const std::size_t required = length + 2U;
    if (required + 1U > capacity) return 0U;
    std::memcpy(destination, body, length);
    destination[length] = '\r';
    destination[length + 1U] = '\n';
    destination[length + 2U] = '\0';
    return length + 2U;
}

std::uint8_t Um982Protocol::nmea_xor(const char *data,
                                      std::size_t length) noexcept
{
    // NMEA 校验公式：checksum = data[0] XOR ... XOR data[length-1]。
    std::uint8_t checksum = 0U;
    for (std::size_t index = 0U; index < length; ++index) {
        checksum ^= static_cast<std::uint8_t>(data[index]);
    }
    return checksum;
}

std::uint32_t Um982Protocol::unicore_crc32(const char *data,
                                            std::size_t length) noexcept
{
    // 初值为 0、逐字节低位优先，反射多项式 0xEDB88320；每字节迭代 8 位。
    std::uint32_t crc = 0U;
    for (std::size_t index = 0U; index < length; ++index) {
        crc ^= static_cast<std::uint8_t>(data[index]);
        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320U
                                   : crc >> 1U;
        }
    }
    return crc;
}

std::uint64_t Um982Protocol::utc_usec(std::uint32_t date_ddmmyy,
                                       std::uint32_t time_hhmmss_ms) noexcept
{
    // NMEA 两位年份按 [80,99]->19xx、[00,79]->20xx 映射；先严格验证闰年、
    // 月天数和时分秒，再计算 (days*86400+h*3600+m*60+s)*1e6+ms*1e3。
    const unsigned day = date_ddmmyy / 10000U;
    const unsigned month = (date_ddmmyy / 100U) % 100U;
    const unsigned short_year = date_ddmmyy % 100U;
    const int year = short_year >= 80U ? 1900 + static_cast<int>(short_year)
                                       : 2000 + static_cast<int>(short_year);
    const std::uint32_t packed_time = time_hhmmss_ms / 1000U;
    const unsigned hours = packed_time / 10000U;
    const unsigned minutes = (packed_time / 100U) % 100U;
    const unsigned seconds = packed_time % 100U;
    const unsigned millis = time_hhmmss_ms % 1000U;
    if (month < 1U || month > 12U || day < 1U || day > 31U ||
        hours >= 24U || minutes >= 60U || seconds >= 60U ||
        day > (month == 2U ? (leap_year(year) ? 29U : 28U)
                           : ((month == 4U || month == 6U || month == 9U ||
                               month == 11U) ? 30U : 31U))) {
        return 0U;
    }
    const std::int64_t days = days_from_civil(year, month, day);
    if (days < 0) return 0U;
    return (static_cast<std::uint64_t>(days) * 86400ULL +
            static_cast<std::uint64_t>(hours) * 3600ULL +
            static_cast<std::uint64_t>(minutes) * 60ULL + seconds) *
               1000000ULL +
           static_cast<std::uint64_t>(millis) * 1000ULL;
}

} // namespace dima::protocols::um982
