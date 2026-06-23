#include "sensor_packet.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace vigia {
namespace {

bool startsWith(std::string_view line, std::string_view prefix) {
    return line.size() >= prefix.size() &&
           line.compare(0, prefix.size(), prefix) == 0;
}

/** Returns true when v is finite (not NaN, not Inf). */
bool isFiniteVal(float v) { return std::isfinite(v); }
bool isFiniteVal(double v) { return std::isfinite(v); }

} // namespace

std::optional<ImuSample> parseImuLine(std::string_view line) {
    if (!startsWith(line, "VIGIA_IMU "))
        return std::nullopt;

    const std::string buf(line);

    ImuSample sample{};
    unsigned cal = 0;
    unsigned valid = 0;
    unsigned long long timestamp_us = 0;
    unsigned seq = 0;

    const int matched = std::sscanf(
        buf.c_str(),
        "VIGIA_IMU seq=%u timestamp_us=%llu qw=%f qx=%f qy=%f qz=%f "
        "ax=%f ay=%f az=%f cal=%u valid=%u qnorm=%f",
        &seq,
        &timestamp_us,
        &sample.qw,
        &sample.qx,
        &sample.qy,
        &sample.qz,
        &sample.ax,
        &sample.ay,
        &sample.az,
        &cal,
        &valid,
        &sample.qnorm);

    if (matched < 11)
        return std::nullopt;

    sample.seq = static_cast<uint32_t>(seq);
    sample.timestamp_us = timestamp_us;
    sample.cal_status = static_cast<uint8_t>(cal);
    sample.valid = valid != 0;

    if (matched < 12)
        sample.qnorm = 0.0f;

    // Reject NaN / Inf in any float field — a faulty IMU or corrupted UART byte
    // could produce these and silently poison the fusion score or HMAC payload.
    if (!isFiniteVal(sample.qw) || !isFiniteVal(sample.qx) ||
        !isFiniteVal(sample.qy) || !isFiniteVal(sample.qz) ||
        !isFiniteVal(sample.ax) || !isFiniteVal(sample.ay) ||
        !isFiniteVal(sample.az) || !isFiniteVal(sample.qnorm))
        return std::nullopt;

    return sample;
}

std::optional<GpsFix> parseGpsLine(std::string_view line) {
    if (!startsWith(line, "VIGIA_GPS "))
        return std::nullopt;

    const std::string buf(line);

    GpsFix fix{};
    unsigned fix_type = 0;
    unsigned satellites = 0;
    unsigned valid = 0;
    unsigned seq = 0;
    unsigned long long timestamp_us = 0;
    char source[32]{};

    int matched = std::sscanf(
        buf.c_str(),
        "VIGIA_GPS seq=%u timestamp_us=%llu lat=%lf lon=%lf speed_ms=%f "
        "fix_type=%u satellites=%u hdop=%f valid=%u src=%31s",
        &seq,
        &timestamp_us,
        &fix.latitude,
        &fix.longitude,
        &fix.speed_ms,
        &fix_type,
        &satellites,
        &fix.hdop,
        &valid,
        source);

    if (matched < 9) {
        matched = std::sscanf(
            buf.c_str(),
            "VIGIA_GPS seq=%u lat=%lf lon=%lf speed_ms=%f fix_type=%u "
            "satellites=%u hdop=%f valid=%u",
            &seq,
            &fix.latitude,
            &fix.longitude,
            &fix.speed_ms,
            &fix_type,
            &satellites,
            &fix.hdop,
            &valid);
        if (matched < 8)
            return std::nullopt;
    } else {
        fix.source = source;
        fix.timestamp_us = timestamp_us;
    }

    fix.seq = static_cast<uint32_t>(seq);
    fix.fix_type = static_cast<uint8_t>(fix_type);
    fix.satellites = static_cast<uint8_t>(satellites);
    fix.valid = valid != 0;

    // Reject physically impossible or non-finite coordinates.
    if (!isFiniteVal(fix.latitude)  || !isFiniteVal(fix.longitude) ||
        !isFiniteVal(fix.speed_ms) || !isFiniteVal(fix.hdop) ||
        fix.latitude  < -90.0  || fix.latitude  > 90.0 ||
        fix.longitude < -180.0 || fix.longitude > 180.0 ||
        fix.speed_ms  < 0.0f   ||
        fix.hdop      < 0.0f)
        return std::nullopt;

    return fix;
}

std::optional<PingReport> parsePingLine(std::string_view line) {
    if (!startsWith(line, "VIGIA_PING "))
        return std::nullopt;

    const std::string buf(line);

    PingReport ping{};
    unsigned seq = 0;
    unsigned long long uptime_ms = 0;
    unsigned long long boot_ms = 0;

    const int matched = std::sscanf(
        buf.c_str(),
        "VIGIA_PING seq=%u uptime_ms=%llu boot_ms=%llu",
        &seq,
        &uptime_ms,
        &boot_ms);

    if (matched < 2)
        return std::nullopt;

    ping.seq = static_cast<uint32_t>(seq);
    ping.uptime_ms = uptime_ms;
    if (matched >= 3)
        ping.boot_ms = boot_ms;

    return ping;
}

} // namespace vigia
