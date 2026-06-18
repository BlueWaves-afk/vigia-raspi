#pragma once
//
// ble_frame_codec.hpp — VIGIA BLE telemetry wire-format codec.
//
// This is the Pi-side encoder for the frame the Android app decodes in
// core/sensor/.../ble/BleDataStreamerImpl.kt. The two MUST stay byte-identical.
//
// Wire format (little-endian throughout):
//   [0]       uint8    version = 0x01
//   [1..4]    float32  RRI score, clamped to [0,1]
//   [5]       uint8    dims code  (see DimsCode)
//   [6..]     float32[dims]  spatial latent vector
//
//   dims code 0x00 -> 256-D  (frame = 6 + 256*4 = 1030 bytes)
//   dims code 0x01 -> 512-D  (frame = 6 + 512*4 = 2054 bytes)
//   dims code 0xFF -> RRI-only, NO vector (frame = 6 bytes)
//
// The 0xFF sentinel is the graceful-degradation mode (design spec §7.1):
// when the BLE link is poor we keep streaming RRI but drop the vector.
// The Android decoder must accept 0xFF (currently rejects unknown codes —
// see docs/app_dashcam_integration.md §7.1, the one required app change).
//
// Header-only, no ROS2 / OpenCV / BlueZ dependency on purpose: it is unit-
// testable on the host with plain g++/clang. See test/test_ble_frame_codec.cpp.
//

#include <cstdint>
#include <cstring>
#include <vector>

namespace vigia::ble {

// IEEE-754 + little-endian assumptions. aarch64 (Pi 5) and Android are both LE.
static_assert(sizeof(float) == 4, "float must be 32-bit IEEE-754");

inline constexpr uint8_t kFrameVersion = 0x01;
inline constexpr std::size_t kHeaderBytes = 6;  // version(1) + rri(4) + dims(1)

enum class DimsCode : uint8_t {
    k256     = 0x00,
    k512     = 0x01,
    kRriOnly = 0xFF,
};

// Returns the number of latent floats for a dims code (0 for RRI-only / unknown).
inline std::size_t dims_for_code(uint8_t code) {
    switch (code) {
        case static_cast<uint8_t>(DimsCode::k256): return 256;
        case static_cast<uint8_t>(DimsCode::k512): return 512;
        default:                                   return 0;  // 0xFF or unknown
    }
}

// Total wire size for a given dims code.
inline std::size_t frame_size_for_code(uint8_t code) {
    return kHeaderBytes + dims_for_code(code) * sizeof(float);
}

// ── float clamp (no <algorithm> needed) ────────────────────────────────────
inline float clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

// ── Encoder ────────────────────────────────────────────────────────────────
//
// Encodes into `out`, resizing it to the exact frame length. The latent vector
// is truncated or zero-padded to match `code`'s dimension. RRI is clamped to
// [0,1] (the app silently discards out-of-range frames, so we never send them).
//
// For kRriOnly, `latent`/`latent_len` are ignored.
//
// Returns the number of bytes written.
inline std::size_t encode_frame(float rri,
                                DimsCode code,
                                const float * latent,
                                std::size_t latent_len,
                                std::vector<uint8_t> & out)
{
    const uint8_t code_u = static_cast<uint8_t>(code);
    const std::size_t dims = dims_for_code(code_u);
    const std::size_t total = kHeaderBytes + dims * sizeof(float);
    out.resize(total);

    out[0] = kFrameVersion;
    const float rri_c = clamp01(rri);
    std::memcpy(out.data() + 1, &rri_c, sizeof(float));  // LE on target platforms
    out[5] = code_u;

    if (dims > 0) {
        float * dst = reinterpret_cast<float *>(out.data() + kHeaderBytes);
        const std::size_t n = (latent_len < dims) ? latent_len : dims;
        if (latent && n > 0) std::memcpy(dst, latent, n * sizeof(float));
        // Zero-pad the tail if the source vector was shorter than `dims`.
        for (std::size_t i = n; i < dims; ++i) dst[i] = 0.0f;
    }
    return total;
}

// ── Decoder (mirror of the Kotlin decoder — used by host tests) ─────────────
struct DecodedFrame {
    bool   valid{false};
    float  rri{0.0f};
    uint8_t dims_code{0};
    std::vector<float> latent;  // empty for RRI-only
};

// Mirrors BleDataStreamerImpl.toTelemetryFrame(): silently rejects malformed
// frames by returning {valid=false}. Accepts the 0xFF RRI-only sentinel.
inline DecodedFrame decode_frame(const uint8_t * data, std::size_t len)
{
    DecodedFrame f;
    if (len < kHeaderBytes)       return f;          // too short
    if (data[0] != kFrameVersion) return f;          // bad version

    float rri;
    std::memcpy(&rri, data + 1, sizeof(float));
    if (rri < 0.0f || rri > 1.0f) return f;          // out-of-range RRI

    const uint8_t code = data[5];
    if (code != static_cast<uint8_t>(DimsCode::k256) &&
        code != static_cast<uint8_t>(DimsCode::k512) &&
        code != static_cast<uint8_t>(DimsCode::kRriOnly)) {
        return f;                                    // unknown dims code
    }

    const std::size_t dims = dims_for_code(code);
    const std::size_t expected = kHeaderBytes + dims * sizeof(float);
    if (len < expected) return f;                    // truncated payload

    f.rri = rri;
    f.dims_code = code;
    if (dims > 0) {
        f.latent.resize(dims);
        std::memcpy(f.latent.data(), data + kHeaderBytes, dims * sizeof(float));
    }
    f.valid = true;
    return f;
}

}  // namespace vigia::ble
