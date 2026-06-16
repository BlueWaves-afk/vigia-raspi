#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace vigia {

struct ImuSample {
    uint32_t seq{0};
    uint64_t timestamp_us{0};
    float qw{0.0f};
    float qx{0.0f};
    float qy{0.0f};
    float qz{0.0f};
    float ax{0.0f};
    float ay{0.0f};
    float az{0.0f};
    uint8_t cal_status{0};
    bool valid{false};
    float qnorm{0.0f};
};

struct GpsFix {
    uint32_t seq{0};
    uint64_t timestamp_us{0};
    double latitude{0.0};
    double longitude{0.0};
    float speed_ms{0.0f};
    uint8_t fix_type{0};
    uint8_t satellites{0};
    float hdop{0.0f};
    bool valid{false};
    std::string source;
};

struct PingReport {
    uint32_t seq{0};
    uint64_t uptime_ms{0};
    uint64_t boot_ms{0};
};

struct SensorHealth {
    uint64_t imu_count{0};
    uint64_t gps_count{0};
    uint64_t ping_count{0};
    uint64_t imu_seq_gaps{0};
    uint64_t gps_seq_gaps{0};
    uint64_t parse_errors{0};
    uint64_t last_ping_uptime_ms{0};
    uint32_t last_imu_seq{0};
    uint32_t last_gps_seq{0};
    bool have_imu_seq{false};
    bool have_gps_seq{false};
};

std::optional<ImuSample> parseImuLine(std::string_view line);
std::optional<GpsFix> parseGpsLine(std::string_view line);
std::optional<PingReport> parsePingLine(std::string_view line);

} // namespace vigia
