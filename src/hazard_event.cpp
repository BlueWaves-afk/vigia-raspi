#include "hazard_event.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace vigia {

namespace {

std::string formatObservedAtIso8601()
{
    const auto now = std::chrono::system_clock::now();
    const auto secs =
        std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) -
        std::chrono::duration_cast<std::chrono::milliseconds>(secs);

    const std::time_t t = secs.count();
    std::tm tm_buf{};
#if defined(_WIN32)
    gmtime_s(&tm_buf, &t);
#else
    gmtime_r(&t, &tm_buf);
#endif

    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);

    std::ostringstream oss;
    oss << buf << '.' << std::setw(3) << std::setfill('0') << ms.count() << 'Z';
    return oss.str();
}

} // namespace

std::string uuidBytesToString(const uint8_t id[16])
{
    char buf[37];
    std::snprintf(buf, sizeof(buf),
                  "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                  id[0], id[1], id[2], id[3],
                  id[4], id[5], id[6], id[7],
                  id[8], id[9], id[10], id[11],
                  id[12], id[13], id[14], id[15]);
    return std::string(buf);
}

std::string hazardObservationToJson(const HazardObservation& obs)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    const std::string eventId = uuidBytesToString(obs.event_id);
    const std::string observedAt = formatObservedAtIso8601();

    oss << '{'
        << "\"event_id\":\"" << eventId << "\","
        << "\"device_id\":\"" << obs.device_id << "\","
        << "\"device_seq\":" << obs.device_seq << ','
        << "\"observed_at\":\"" << observedAt << "\","
        << "\"hazard_class\":" << static_cast<unsigned>(obs.hazard_class) << ','
        << "\"location\":{\"lat\":" << obs.lat << ",\"lon\":" << obs.lon << "},"
        << "\"hazard\":{"
        << "\"rri\":" << obs.rri << ','
        << "\"iss\":" << obs.iss << ','
        << "\"yolo_conf\":" << obs.yolo_conf << ','
        << "\"geometry_conf\":" << obs.geometry_conf << ','
        << "\"temporal_conf\":" << obs.temporal_conf << ','
        << "\"bbox\":[" << obs.bbox_x << ',' << obs.bbox_y << ','
        << obs.bbox_w << ',' << obs.bbox_h << "],"
        << "\"frame_index\":" << obs.frame_index
        << "},"
        << "\"motion\":{"
        << "\"speed_mps\":" << obs.speed_ms << ','
        << "\"hdop\":" << obs.hdop << ','
        << "\"fix_type\":" << static_cast<unsigned>(obs.gps_fix_type)
        << "}"
        << '}';

    return oss.str();
}

} // namespace vigia
