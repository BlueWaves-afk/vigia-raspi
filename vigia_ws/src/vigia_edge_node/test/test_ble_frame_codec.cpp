// Host-compilable unit test for ble_frame_codec.hpp — no ROS2/BlueZ needed.
//   g++ -std=c++17 -I../include test_ble_frame_codec.cpp -o /tmp/t && /tmp/t
#include "vigia_edge_node/ble_frame_codec.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace vigia::ble;

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

int main() {
    // ── frame sizes match the spec ──────────────────────────────────────────
    CHECK(frame_size_for_code(0x00) == 1030);   // 256-D
    CHECK(frame_size_for_code(0x01) == 2054);   // 512-D
    CHECK(frame_size_for_code(0xFF) == 6);      // RRI-only

    // ── 256-D round trip ────────────────────────────────────────────────────
    {
        std::vector<float> v(256);
        for (int i = 0; i < 256; ++i) v[i] = static_cast<float>(i) * 0.5f - 30.0f;
        std::vector<uint8_t> buf;
        std::size_t n = encode_frame(0.42f, DimsCode::k256, v.data(), v.size(), buf);
        CHECK(n == 1030);
        CHECK(buf[0] == 0x01);
        CHECK(buf[5] == 0x00);

        DecodedFrame d = decode_frame(buf.data(), buf.size());
        CHECK(d.valid);
        CHECK(std::fabs(d.rri - 0.42f) < 1e-6f);
        CHECK(d.dims_code == 0x00);
        CHECK(d.latent.size() == 256);
        for (int i = 0; i < 256; ++i) CHECK(std::fabs(d.latent[i] - v[i]) < 1e-6f);
    }

    // ── 512-D round trip ────────────────────────────────────────────────────
    {
        std::vector<float> v(512, 1.25f);
        std::vector<uint8_t> buf;
        encode_frame(1.0f, DimsCode::k512, v.data(), v.size(), buf);
        DecodedFrame d = decode_frame(buf.data(), buf.size());
        CHECK(d.valid);
        CHECK(d.latent.size() == 512);
        CHECK(std::fabs(d.rri - 1.0f) < 1e-6f);
    }

    // ── RRI-only sentinel (0xFF) ────────────────────────────────────────────
    {
        std::vector<uint8_t> buf;
        std::size_t n = encode_frame(0.9f, DimsCode::kRriOnly, nullptr, 0, buf);
        CHECK(n == 6);
        DecodedFrame d = decode_frame(buf.data(), buf.size());
        CHECK(d.valid);
        CHECK(d.dims_code == 0xFF);
        CHECK(d.latent.empty());
        CHECK(std::fabs(d.rri - 0.9f) < 1e-6f);
    }

    // ── RRI clamping: out-of-range input is clamped before send ──────────────
    {
        std::vector<float> v(256, 0.0f);
        std::vector<uint8_t> buf;
        encode_frame(5.0f, DimsCode::k256, v.data(), v.size(), buf);   // > 1
        DecodedFrame d = decode_frame(buf.data(), buf.size());
        CHECK(d.valid);                       // clamped to 1.0 -> in range -> valid
        CHECK(std::fabs(d.rri - 1.0f) < 1e-6f);

        encode_frame(-2.0f, DimsCode::k256, v.data(), v.size(), buf);  // < 0
        d = decode_frame(buf.data(), buf.size());
        CHECK(d.valid);
        CHECK(std::fabs(d.rri - 0.0f) < 1e-6f);
    }

    // ── short vector is zero-padded to the declared dimension ───────────────
    {
        std::vector<float> v(10, 7.0f);
        std::vector<uint8_t> buf;
        encode_frame(0.5f, DimsCode::k256, v.data(), v.size(), buf);
        DecodedFrame d = decode_frame(buf.data(), buf.size());
        CHECK(d.valid && d.latent.size() == 256);
        for (int i = 0; i < 10; ++i)  CHECK(std::fabs(d.latent[i] - 7.0f) < 1e-6f);
        for (int i = 10; i < 256; ++i) CHECK(d.latent[i] == 0.0f);
    }

    // ── malformed frames are rejected ───────────────────────────────────────
    {
        std::vector<uint8_t> tooShort(3, 0);
        CHECK(!decode_frame(tooShort.data(), tooShort.size()).valid);

        std::vector<uint8_t> badVer(1030, 0);
        badVer[0] = 0x02;  // wrong version
        CHECK(!decode_frame(badVer.data(), badVer.size()).valid);

        std::vector<uint8_t> badDims(1030, 0);
        badDims[0] = 0x01; badDims[5] = 0x7E;  // unknown dims code
        // rri bytes are 0.0 -> in range; should still reject on dims code
        CHECK(!decode_frame(badDims.data(), badDims.size()).valid);

        // declared 512-D but truncated buffer
        std::vector<uint8_t> truncated(100, 0);
        truncated[0] = 0x01; truncated[5] = 0x01;
        CHECK(!decode_frame(truncated.data(), truncated.size()).valid);
    }

    if (failures == 0) std::printf("ble_frame_codec: ALL TESTS PASSED\n");
    else               std::printf("ble_frame_codec: %d FAILURE(S)\n", failures);
    return failures ? 1 : 0;
}
