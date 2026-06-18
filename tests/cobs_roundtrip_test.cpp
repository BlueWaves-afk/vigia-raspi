/**
 * COBS round-trip + SignedEtPacket layout test (host-side).
 *
 * clang++ -std=c++17 tests/cobs_roundtrip_test.cpp src/cobs.cpp \
 *   src/signed_et_packet.cpp -Iinclude -O2 -o cobs_roundtrip_test
 */

#include "cobs.hpp"
#include "signed_et_packet.hpp"

#include <cstring>
#include <iostream>

using namespace vigia;

namespace {

int failures = 0;

void expectTrue(bool cond, const char* label) {
    if (cond)
        std::cout << "[PASS] " << label << '\n';
    else {
        std::cout << "[FAIL] " << label << '\n';
        ++failures;
    }
}

} // namespace

int main() {
    std::cout << "[TEST] COBS round-trip + SignedEtPacket\n";

    SignedEtPacketView pkt{};
    pkt.magic = kSignedEtMagic;
    pkt.version = kSignedEtVersion;
    pkt.timestamp_us = 123456789ULL;
    pkt.sequence = 42;
    pkt.qw = 0.99f;
    pkt.latitude = 37.1234567;
    pkt.longitude = -122.1234567;
    pkt.speed_ms = 8.3f;
    pkt.fix_type = 3;
    pkt.satellites = 12;
    std::memset(pkt.et_hash, 0xAB, 32);
    std::memset(pkt.ecdsa_sig, 0xCD, 64);

    std::uint8_t raw[173];
    std::memcpy(raw, &pkt, sizeof(pkt));
    expectTrue(sizeof(pkt) == kSignedEtPacketSize, "struct size 173");

    std::uint8_t frame[256];
    const std::size_t frame_len = cobsEncode(raw, sizeof(raw), frame, sizeof(frame));
    expectTrue(frame_len > sizeof(raw), "COBS frame larger than payload");

    std::uint8_t decoded[256];
    const std::size_t dec_len = cobsDecode(frame + 1, frame_len - 2, decoded, sizeof(decoded));
    expectTrue(dec_len == kSignedEtPacketSize, "decode length");

    SignedEtSample sample{};
    expectTrue(parseSignedEtPacket(decoded, dec_len, sample), "parseSignedEtPacket");
    expectTrue(sample.sequence == 42, "sequence preserved");
    expectTrue(sample.et_hash[0] == 0xAB, "hash preserved");

    if (failures == 0) {
        std::cout << "[TEST] All COBS tests passed\n";
        return 0;
    }
    std::cout << "[TEST] " << failures << " failure(s)\n";
    return 1;
}
