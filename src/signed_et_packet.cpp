#include "signed_et_packet.hpp"

#include <cmath>
#include <cstring>

namespace vigia {

bool parseSignedEtPacket(const std::uint8_t* data, std::size_t len,
                         SignedEtSample& out)
{
    if (!data || len != kSignedEtPacketSize)
        return false;

    SignedEtPacketView pkt{};
    std::memcpy(&pkt, data, sizeof(pkt));

    if (pkt.magic != kSignedEtMagic || pkt.version != kSignedEtVersion)
        return false;

    // Reject non-finite or physically impossible IMU / GPS values in binary
    // packets — a corrupt serial byte could flip a float to NaN or Inf.
    if (!std::isfinite(pkt.qw) || !std::isfinite(pkt.qx) ||
        !std::isfinite(pkt.qy) || !std::isfinite(pkt.qz) ||
        !std::isfinite(pkt.ax) || !std::isfinite(pkt.ay) ||
        !std::isfinite(pkt.az))
        return false;

    if (!std::isfinite(pkt.latitude)  || !std::isfinite(pkt.longitude) ||
        !std::isfinite(pkt.speed_ms)  ||
        pkt.latitude  < -90.0  || pkt.latitude  > 90.0 ||
        pkt.longitude < -180.0 || pkt.longitude > 180.0 ||
        pkt.speed_ms  < 0.0f)
        return false;

    out.timestamp_us = pkt.timestamp_us;
    out.sequence = pkt.sequence;
    out.qw = pkt.qw;
    out.qx = pkt.qx;
    out.qy = pkt.qy;
    out.qz = pkt.qz;
    out.ax = pkt.ax;
    out.ay = pkt.ay;
    out.az = pkt.az;
    out.cal_status = pkt.cal_status;
    out.latitude = pkt.latitude;
    out.longitude = pkt.longitude;
    out.speed_ms = pkt.speed_ms;
    out.fix_type = pkt.fix_type;
    out.satellites = pkt.satellites;
    std::memcpy(out.et_hash.data(), pkt.et_hash, 32);
    std::memcpy(out.ecdsa_sig.data(), pkt.ecdsa_sig, 64);
    return true;
}

} // namespace vigia
