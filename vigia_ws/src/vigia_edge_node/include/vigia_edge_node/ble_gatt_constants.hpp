#pragma once
//
// ble_gatt_constants.hpp — Pi-side GATT profile constants.
//
// MUST stay byte-identical with the Android app's GattConstants.kt:
//   vigia2/core/sensor/.../ble/GattConstants.kt
//
// UUIDs are the final random 128-bit values (decision D6,
// docs/app_dashcam_integration.md). Generated 2026-06.
//

#include <cstdint>

namespace vigia::ble {

// ── Service & characteristic UUIDs (string form for BlueZ D-Bus) ────────────
inline constexpr const char * kServiceUuid   = "5e355f98-eabf-4ae0-8417-919e926d411e";
inline constexpr const char * kHandshakeUuid = "eb4b161b-3be6-4719-aa0f-8ef40bd44a36";  // Read+Write+Notify
inline constexpr const char * kTelemetryUuid = "4d231514-5514-4847-bb6d-64e3aa7a3ffb";  // Notify
inline constexpr const char * kControlUuid   = "0bb821dd-6b24-4185-ad69-662510769d19";  // Write (phone->Pi)
inline constexpr const char * kAttestUuid    = "580c5fb6-5283-4194-84c8-5d6aec75b88a";  // Notify (anti-spoof)
inline constexpr const char * kResponseUuid  = "a3f1c2d7-88e4-4b9a-b3c1-5d7e9f012345";  // Notify (Pi->phone control ACK)
inline constexpr const char * kCccdUuid      = "00002902-0000-1000-8000-00805f9b34fb";

// ── Handshake protocol bytes (mirror GattConstants.Protocol) ────────────────
// NOTE: the legacy HMAC step bytes are kept for wire compatibility, but the
// payloads change under the ECDH re-architecture (design spec §4.2). See that
// section: CHALLENGE/RESPONSE now carry P-256 pubkeys + signatures, not an HMAC.
namespace proto {
    inline constexpr uint8_t kHello     = 0x01;  // phone -> Pi
    inline constexpr uint8_t kChallenge = 0x02;  // Pi -> phone  [0x02 | nonce | Pi_pub | sig]
    inline constexpr uint8_t kResponse  = 0x03;  // phone -> Pi  [0x03 | nonce | Phone_pub | sig]
    inline constexpr uint8_t kBound     = 0x04;  // Pi -> phone  (CONFIRM on success)
    inline constexpr uint8_t kErr       = 0xFF;  // Pi -> phone  (failure)
    inline constexpr int     kNonceBytes = 32;
}

// ── CONTROL_CHAR opcodes (phone -> Pi), design spec §7.1 ────────────────────
namespace control {
    inline constexpr uint8_t kRequest256  = 0x10;  // ask for 256-D stream
    inline constexpr uint8_t kRequest512  = 0x11;  // ask for 512-D stream
    inline constexpr uint8_t kPauseStream = 0x12;  // stop telemetry notifications
    inline constexpr uint8_t kResumeStream= 0x13;  // resume telemetry notifications
    inline constexpr uint8_t kRekey       = 0x20;  // rotate session key (§5)
    inline constexpr uint8_t kPing        = 0xF0;  // liveness check
}

// ── RESPONSE_CHAR reply bytes (Pi -> phone), mirrors control opcodes ─────────
namespace response {
    inline constexpr uint8_t kAck  = 0x00;  // [echo_opcode, ACK]
    inline constexpr uint8_t kNack = 0x01;  // [echo_opcode, NACK, reason]
    inline constexpr uint8_t kPong = 0xF1;  // reply to kPing
}

}  // namespace vigia::ble
