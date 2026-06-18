#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace vigia {

#pragma pack(push, 1)
struct SignedEtPacketView {
    std::uint8_t  magic{0};
    std::uint8_t  version{0};
    std::uint64_t timestamp_us{0};
    std::uint32_t sequence{0};
    float         qw{0}, qx{0}, qy{0}, qz{0};
    float         ax{0}, ay{0}, az{0};
    std::uint8_t  cal_status{0};
    std::uint8_t  imu_pad[3]{};
    double        latitude{0};
    double        longitude{0};
    float         speed_ms{0};
    std::uint8_t  fix_type{0};
    std::uint8_t  satellites{0};
    std::uint8_t  gps_pad[1]{};
    std::uint8_t  et_hash[32]{};
    std::uint8_t  ecdsa_sig[64]{};
    std::uint8_t  wire_pad[8]{};
};
#pragma pack(pop)

static_assert(sizeof(SignedEtPacketView) == 173,
              "SignedEtPacketView must match firmware SignedEtPacket");

constexpr std::size_t kSignedEtPacketSize = 173;
constexpr std::uint8_t kSignedEtMagic = 0xE7;
constexpr std::uint8_t kSignedEtVersion = 0x02;

struct SignedEtSample {
    std::uint64_t timestamp_us{0};
    std::uint32_t sequence{0};
    float qw{0}, qx{0}, qy{0}, qz{0};
    float ax{0}, ay{0}, az{0};
    std::uint8_t cal_status{0};
    double latitude{0};
    double longitude{0};
    float speed_ms{0};
    std::uint8_t fix_type{0};
    std::uint8_t satellites{0};
    std::array<std::uint8_t, 32> et_hash{};
    std::array<std::uint8_t, 64> ecdsa_sig{};
    bool sig_valid{false};
};

bool parseSignedEtPacket(const std::uint8_t* data, std::size_t len,
                         SignedEtSample& out);

} // namespace vigia
