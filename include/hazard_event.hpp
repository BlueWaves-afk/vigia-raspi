#pragma once

#include <cstdint>
#include <string>

namespace vigia {

enum class HazardClass : uint8_t {
    Pothole = 0,
};

/* Hot-path safe — no std::string, no heap allocations in struct. */
struct HazardObservation {
    uint8_t  event_id[16]{};      // UUIDv7 raw bytes
    uint64_t device_seq{0};       // monotonic anti-replay counter
    uint64_t frame_index{0};
    uint64_t timestamp_us{0};     // monotonic boot time (steady clock)
    char     device_id[32]{};     // copied from promoter config on enqueue
    uint8_t  hazard_class{0};     // HazardClass raw value
    float    rri{0.0f};
    float    iss{0.0f};
    float    yolo_conf{0.0f};
    float    geometry_conf{0.0f};
    float    temporal_conf{0.0f};
    int32_t  bbox_x{0};
    int32_t  bbox_y{0};
    int32_t  bbox_w{0};
    int32_t  bbox_h{0};
    double   lat{0.0};
    double   lon{0.0};
    float    speed_ms{0.0f};
    float    hdop{0.0f};
    uint8_t  gps_fix_type{0};
    bool     gps_valid{false};
    bool     signed_et_valid{false};
    uint8_t  et_hash[32]{};       // SHA-256 of EtHashInput when signed_et_valid
    uint32_t signed_et_sequence{0};
};

/* Sync-thread only — may allocate std::string. */
std::string hazardObservationToJson(const HazardObservation& obs);
std::string uuidBytesToString(const uint8_t id[16]);

} // namespace vigia
