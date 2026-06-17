# VIGIA ADAS DePIN Edge Node
## Phase 2 — DePIN Signing Pipeline Contracts
**Document:** `04_phase2_depin_signing_contracts.md`  
**Depends on:** `01_system_architecture_and_roadmap.md`, `02_ros2_node_contracts.md`, `03_pico2_firmware_contracts.md`  
**Status:** PARTIALLY IMPLEMENTED — Software complete; SE hardware wiring pending (see §1)  
**Implementation Status (2026-06-17):**

| Component | File | Status |
|---|---|---|
| `EtHashInput` / `SignedEtPacket` structs | `firmware/src/atecc608a_driver.h` | ✅ Written |
| ATECC608A wrappers + stub mode | `firmware/src/atecc608a_driver.c` | ✅ Written |
| COBS encoder (no-alloc) | `firmware/src/cobs_tx_driver.h/.c` | ✅ Written |
| `no_heap.cpp` operator new ban | `firmware/src/no_heap.cpp` | ✅ Written |
| Phase 2 main loop (COBS tx, EtHashInput populate) | `firmware/src/main.c` | ✅ Written (guarded by `VIGIA_PHASE2=1`) |
| Phase 1 build target | `firmware/CMakeLists.txt` | ✅ Unchanged |
| Phase 2 stub build target | `firmware/CMakeLists.txt` | ✅ Added (`-DVIGIA_BUILD_PHASE2_STUB=ON`) |
| Phase 2 live build target | `firmware/CMakeLists.txt` | ✅ Added (`-DVIGIA_BUILD_PHASE2_LIVE=ON`) |
| `SignedEt.msg` (revised fields) | `vigia_msgs/msg/SignedEt.msg` | ✅ Updated |
| `SensorHealth.msg` | `vigia_msgs/msg/SensorHealth.msg` | ✅ Created |
| Pi COBS decoder | `sensor_bridge_node.cpp` | ✅ Written |
| Pi dual-protocol auto-detection | `sensor_bridge_node.cpp` | ✅ Written |
| Pi IMU history ring buffer (200 samples) | `sensor_bridge_node.hpp/.cpp` | ✅ Written |
| Pi `SensorHealth` publisher (1 Hz) | `sensor_bridge_node.cpp` | ✅ Written |
| Pi select() + chunk read loop | `sensor_bridge_node.cpp` | ✅ Written |
| mbedTLS optional CMake integration | `vigia_edge_node/CMakeLists.txt` | ✅ Added |
| mbedTLS `sig_valid` verify | `sensor_bridge_node.cpp` | ⚠️ TODO stub — requires real SE signatures |  
**Scope:** ATECC608A ECDSA signing on Pico 2 + COBS binary framing + Pi-side COBS parser + mbedTLS sig verification  
**Last updated:** 2026-06-17

---

## 1. Blocker Summary & What's Required from Ben

Phase 2 has **one hard hardware blocker** and **two soft preparation tasks**:

### 1.1 Hard Blocker — ATECC608A Not Yet Wired

The ATECC608A Secure Element is confirmed not yet wired to the Pico 2 bringup board (Ben, 2026-06-17: "i've got to add the SE"). Until the SE is on I2C1 with the correct pull-ups, `atcab_sign()` cannot be tested and the signing pipeline cannot be validated end-to-end.

**Required from Ben:**

| Task | Detail | Required before |
|---|---|---|
| Wire ATECC608A to Pico 2 I2C1 | GP2 = SDA, GP3 = SCL; 4.7 kΩ pull-ups to 3.3 V; ADDR pin to GND (addr 0x60) | Phase 2 firmware ECDSA test |
| Move to zero PCB | Eliminates intermittent breadboard contact — required before any sustained signing validation at 100 Hz IMU rate | Before integration test |
| Interface cam module | CSI camera for Pi 5; out of scope for Phase 2 signing but required for full ADAS demo | Before demo |
| Power distribution board | Stable 5V rail for Pi + Pico + SE; required for reliable I2C at 400 kHz | Before integration test |

### 1.2 What We Can Build Now (No SE Required)

Everything in §3 (Pico 2 firmware) and §4 (Pi-side) except the live `atcab_sign()` call:

| Component | Blocked on SE? | Can build now? |
|---|---|---|
| `EtHashInput` struct population (17 fields) | No | ✅ |
| COBS encoder (`cobs_tx_driver`) | No | ✅ |
| `no_heap.cpp` (operator new override) | No | ✅ |
| `atcab_init()` + `atcab_sha()` — hash only | No | ✅ |
| `atcab_sign()` — ECDSA over secp256r1 | **Yes** | Stub with 0x00×64 |
| Pi-side COBS binary parser in `sensor_bridge_node` | No | ✅ |
| Pi-side mbedTLS `sig_valid` verification | **Yes** (needs real sig) | Stub: always false |
| Pi-side `SignedEt` ROS2 publish | No | ✅ |

---

## 2. Wire Protocol Change: Text → COBS Binary

### Phase 1 (current bringup — in production now)

```
[text line]\n
"VIGIA_IMU seq=N timestamp_us=T qw=W qx=X qy=Y qz=Z ax=A ay=B az=C cal=K valid=V qnorm=R\n"
"VIGIA_GPS seq=N timestamp_us=T lat=L lon=G speed_ms=S fix_type=F satellites=N hdop=H valid=V\n"
"VIGIA_PING uptime_ms=U firmware=V\n"
```

### Phase 2 (target — COBS binary)

```
0x00 [COBS-encoded SignedEtPacket (up to ~175 bytes)] 0x00
```

Both Pi-side parsers MUST be compiled into `sensor_bridge_node.cpp`. At init, the node auto-detects the wire protocol by examining the first non-null byte:
- `V` (0x56) → text mode (bringup)
- Any other byte → COBS binary mode (Phase 2)

This allows the same Pi firmware to work with both Pico 2 firmware revisions without a param change.

---

## 3. Pico 2 Firmware — Phase 2 Additions

### 3.1 Files to Create

```
firmware/src/
├── atecc608a_driver.h    — cryptoauthlib init, sha, sign wrappers
├── atecc608a_driver.c    — atcab_init, atcab_sha, atcab_sign; ATCA_NO_HEAP
├── cobs_tx_driver.h      — encode_cobs(), frame constants
├── cobs_tx_driver.c      — no-alloc static encoder (256-byte working buffer)
└── no_heap.cpp           — operator new/delete = static_assert(false)
```

All existing files (`bno085_driver`, `neo_m8n_driver`, `main.c`) are unmodified in Phase 2.

### 3.2 EtHashInput — 96-byte Packed Struct

```c
// firmware/src/atecc608a_driver.h
#pragma pack(push, 1)
typedef struct {
    uint8_t  device_id[16];       // provisioned in ATECC608A slot 0 at factory
    uint64_t timestamp_us;        // from BNO085 SHTP timestamp (64-bit µs counter)
    uint32_t sequence;            // global monotonic frame counter (increments each super-loop)
    float    qw, qx, qy, qz;     // quaternion from BNO085 Q14 (already converted to float)
    float    ax, ay, az;          // linear accel from BNO085 Q8 (already converted to float)
    uint8_t  cal_status;          // BNO085 calibration status byte
    uint8_t  _pad0[3];            // explicit — align next field to 4 bytes
    double   latitude;            // NEO-M8N NAV-PVT field lon@28 (UBX units: 1e-7 deg)
    double   longitude;           // NEO-M8N NAV-PVT field lat@24
    float    speed_ms;            // NAV-PVT gSpeed@60 (mm/s → m/s)
    uint8_t  fix_type;            // NAV-PVT fixType@20
    uint8_t  satellites;          // NAV-PVT numSV@23
    uint8_t  _pad1[2];            // explicit — end-of-struct alignment
} EtHashInput;                    // sizeof == 96 — verify with static_assert
#pragma pack(pop)
static_assert(sizeof(EtHashInput) == 96, "EtHashInput must be 96 bytes packed");
```

### 3.3 SignedEtPacket — 173-byte Packed Struct

```c
#pragma pack(push, 1)
typedef struct {
    // Header (2 bytes)
    uint8_t  magic;              // 0xE7 — Vigia DePIN frame marker
    uint8_t  version;            // 0x02 — Phase 2 protocol version

    // IMU fields (40 bytes)
    uint64_t timestamp_us;
    uint32_t sequence;
    float    qw, qx, qy, qz;
    float    ax, ay, az;
    uint8_t  cal_status;
    uint8_t  _imu_pad[3];

    // GPS fields (27 bytes)
    double   latitude;
    double   longitude;
    float    speed_ms;
    uint8_t  fix_type;
    uint8_t  satellites;
    uint8_t  _gps_pad[1];

    // Signing fields (96 bytes)
    uint8_t  et_hash[32];        // SHA-256 of EtHashInput via atcab_sha()
    uint8_t  ecdsa_sig[64];      // secp256r1 ECDSA-SHA256 via atcab_sign()
} SignedEtPacket;                // sizeof == 173 — verify with static_assert
#pragma pack(pop)
static_assert(sizeof(SignedEtPacket) == 173, "SignedEtPacket must be 173 bytes packed");
```

### 3.4 COBS Encoder (no-alloc)

```c
// firmware/src/cobs_tx_driver.h
#define COBS_MAX_PAYLOAD 200
#define COBS_FRAME_MAX   (COBS_MAX_PAYLOAD + 2)   // +2 for leading/trailing 0x00

// Returns encoded frame length (including both 0x00 delimiters), or 0 on error.
// out must be at least COBS_FRAME_MAX bytes. Static internal buffer — not reentrant.
size_t encode_cobs(const uint8_t* src, size_t src_len, uint8_t* out, size_t out_cap);
```

```c
// firmware/src/cobs_tx_driver.c
size_t encode_cobs(const uint8_t* src, size_t src_len, uint8_t* out, size_t out_cap) {
    if (!src || !out || src_len == 0 || out_cap < src_len + 2) return 0;

    size_t write = 0;
    out[write++] = 0x00;  // leading delimiter

    size_t code_idx = write++;     // reserve overhead byte position
    uint8_t code = 1;

    for (size_t i = 0; i < src_len; ++i) {
        if (src[i] != 0x00) {
            out[write++] = src[i];
            ++code;
            if (code == 0xFF) {
                out[code_idx] = code;
                code_idx = write++;
                code = 1;
            }
        } else {
            out[code_idx] = code;
            code_idx = write++;
            code = 1;
        }
    }
    out[code_idx] = code;
    out[write++] = 0x00;  // trailing delimiter
    return write;
}
```

### 3.5 ATECC608A Driver Wrappers

```c
// firmware/src/atecc608a_driver.h
#include "cryptoauthlib.h"

// Call once at boot. Returns ATCA_SUCCESS or panics with LED blink code.
ATCA_STATUS vigia_atca_init(void);

// Compute SHA-256 of `input` (len bytes) into `hash_out` (32 bytes).
// Blocking — ~40 ms on I2C 400 kHz. Do NOT call from ISR.
ATCA_STATUS vigia_atca_sha(const uint8_t* input, size_t len, uint8_t* hash_out);

// Sign `hash` (32 bytes) with device private key in slot 0. Output in `sig_out` (64 bytes).
// Blocking — ~57 ms total (includes internal ECDSA computation). Do NOT call from ISR.
ATCA_STATUS vigia_atca_sign(const uint8_t* hash, uint8_t* sig_out);
```

**ATCA_NO_HEAP must be set** in the cryptoauthlib CMakeLists:
```cmake
target_compile_definitions(cryptoauthlib PRIVATE ATCA_NO_HEAP)
```

**I2C1 pin mapping:**
```
GP2 → SDA (I2C1)
GP3 → SCL (I2C1)
I2C addr: 0x60 (ADDR pin → GND)
Speed: 400 kHz (ATECC608A Fast Mode)
Pull-ups: 4.7 kΩ to 3.3 V (external — not on Pico board)
```

### 3.6 Main Loop Integration (main.c — Phase 2 additions)

In the existing super-loop, after every GPS frame received, the main loop now:

```
1. Populate EtHashInput from current BNO085 + NEO-M8N state
2. vigia_atca_sha(&et_hash_input, 96, hash)   // ~40 ms
3. vigia_atca_sign(hash, sig)                  // ~57 ms — total ~97 ms per GPS cycle (1 Hz)
4. Populate SignedEtPacket fields
5. encode_cobs((uint8_t*)&pkt, 173, cobs_buf, sizeof(cobs_buf))
6. tud_cdc_write(cobs_buf, cobs_len)
7. tud_cdc_write_flush()
```

**Timing note:** signing adds ~97 ms per GPS cycle. GPS fires at 1 Hz → signing budget is 1000 ms. Total signing budget used: ~10%. Acceptable. IMU loop (100 Hz, 10 ms budget) is unaffected — signing runs synchronously in the GPS branch only.

### 3.7 Stub Mode (before SE is wired)

When `VIGIA_PHASE2_STUB=1` is defined at compile time, `vigia_atca_sha()` and `vigia_atca_sign()` are no-ops that zero-fill their outputs. This lets the COBS framing and Pi-side parser be validated end-to-end before Ben wires the SE.

```c
// atecc608a_driver.c (stub section)
#ifdef VIGIA_PHASE2_STUB
ATCA_STATUS vigia_atca_init(void) { return ATCA_SUCCESS; }
ATCA_STATUS vigia_atca_sha(const uint8_t* in, size_t len, uint8_t* out) {
    (void)in; (void)len; memset(out, 0x00, 32); return ATCA_SUCCESS;
}
ATCA_STATUS vigia_atca_sign(const uint8_t* hash, uint8_t* sig) {
    (void)hash; memset(sig, 0x00, 64); return ATCA_SUCCESS;
}
#endif
```

---

## 4. Pi-Side — sensor_bridge_node Changes (Phase 2)

### 4.1 Read Loop — Dual-Protocol Auto-Detection

Replace the current line-based `read_loop()` with a framed COBS reader that falls back to text mode:

```
State: DETECT (read first byte)
  → 'V' (0x56): enter TEXT mode (existing line reader)
  → 0x00: enter COBS mode (read until next 0x00, decode, dispatch)
  → restart on timeout
```

### 4.2 COBS Decoder (Pi-side, in sensor_bridge_node.cpp)

```cpp
// Returns decoded length or 0 on error. dst must be >= src_len.
static size_t decode_cobs(const uint8_t* src, size_t src_len, uint8_t* dst) {
    if (!src || !dst || src_len < 2) return 0;
    size_t write = 0, read = 0;
    while (read < src_len) {
        uint8_t code = src[read++];
        if (code == 0) break;
        for (uint8_t i = 1; i < code && read < src_len; ++i)
            dst[write++] = src[read++];
        if (code < 0xFF && read < src_len)
            dst[write++] = 0x00;
    }
    return write;
}
```

### 4.3 SignedEt Dispatch

After COBS decode, verify:
1. `decoded_len == 173`
2. `pkt->magic == 0xE7`
3. `pkt->version == 0x02`
4. Anti-replay: `pkt->sequence > last_et_seq_`

Then build `vigia_msgs::msg::SignedEt`:
```cpp
auto msg = std::make_unique<vigia_msgs::msg::SignedEt>();
msg->header.stamp    = now();
msg->timestamp_us    = pkt->timestamp_us;
msg->sequence        = pkt->sequence;
// ... populate all IMU + GPS fields from pkt ...
std::copy(pkt->et_hash,   pkt->et_hash   + 32, msg->et_hash.begin());
std::copy(pkt->ecdsa_sig, pkt->ecdsa_sig + 64, msg->ecdsa_sig.begin());
msg->sig_valid = false;  // Phase 2 stub — set true after mbedTLS verify (§4.4)
pub_et_->publish(std::move(msg));
```

### 4.4 mbedTLS Signature Verification (Phase 2 — requires real SE)

When Ben's SE is wired and producing real ECDSA signatures:

```cpp
// In process_signed_et():
mbedtls_ecdsa_context ctx;
mbedtls_ecdsa_init(&ctx);
// Load device public key (provisioned at factory, stored in /etc/vigia/device_pubkey.pem)
// mbedtls_ecdsa_read_public(ctx, pubkey_der, pubkey_der_len);
// int ret = mbedtls_ecdsa_verify(&ctx, pkt->et_hash, 32, &sig_point);
// msg->sig_valid = (ret == 0);
```

**Dependency:** `libmbedtls-dev` on Pi OS — already available. Add to `CMakeLists.txt`:
```cmake
find_package(MbedTLS REQUIRED)
target_link_libraries(vigia_edge_node_exe PRIVATE MbedTLS::mbedcrypto)
```

---

## 5. vigia_msgs — SignedEt Message Definition

**File:** `vigia_ws/src/vigia_msgs/msg/SignedEt.msg`

```
# DePIN signed event telemetry packet
std_msgs/Header header
uint64 timestamp_us
uint32 sequence

# IMU
float32 q_w
float32 q_x
float32 q_y
float32 q_z
float32 lin_accel_x
float32 lin_accel_y
float32 lin_accel_z
uint8   calibration_status

# GPS
float64 lat
float64 lon
float32 speed_ms
uint8   fix_type
uint8   satellites_used
float32 hdop
bool    valid_fix

# Signing
uint8[32] et_hash
uint8[64] ecdsa_sig
bool      sig_valid       # false until SE wired and mbedTLS verify passes
```

---

## 6. Teammate Integration — Ben's SensorBridge vs Our SensorBridgeNode

Ben's standalone `src/sensor_bridge.cpp` / `include/sensor_state.hpp` has two features our ROS2 node currently lacks:

### 6.1 IMU History Ring Buffer (200 samples)

`SensorState::getSampleAtOrBefore(uint64_t timestamp_us)` — reverse-scan over 200-sample history for GPS-timestamp-aligned IMU lookup. Critical for Phase 2: the `EtHashInput` uses Pico's `timestamp_us`, and the Pi must find the corresponding IMU sample when assembling a `HazardEvent`.

**Action:** Add to `sensor_bridge_node.hpp`:
```cpp
static constexpr size_t kImuHistorySize = 200;
struct ImuSnapshot {
    uint64_t timestamp_us{0};
    float qw{},qx{},qy{},qz{},ax{},ay{},az{};
};
std::array<ImuSnapshot, kImuHistorySize> imu_history_{};
size_t imu_history_head_{0};
std::mutex imu_history_mutex_;
// Returns closest IMU sample at or before target_us, or nullopt if history empty
std::optional<ImuSnapshot> imu_at_or_before(uint64_t target_us) const;
```

Expose as a ROS2 service `/vigia/imu_at_timestamp` for FusionNode queries.

### 6.2 SensorHealth Diagnostics

Ben's `SensorHealth` struct (imu_count, gps_count, ping_count, imu_seq_gaps, gps_seq_gaps, parse_errors) should be published as `/vigia/sensor_health` on a `vigia_msgs::msg::SensorHealth` topic at 1 Hz.

**Action:** Create `vigia_msgs/msg/SensorHealth.msg`:
```
uint64 imu_count
uint64 gps_count
uint64 ping_count
uint32 imu_seq_gaps
uint32 gps_seq_gaps
uint32 parse_errors
float32 imu_hz_measured
```

### 6.3 Read Loop Efficiency

Ben uses `select()` + 256-byte chunk reads + `string::find('\n')` scan — cuts syscall rate 256× vs our byte-by-byte `::read()`. Adopt in Phase 2 rewrite of `read_loop()`.

---

## 7. Integration Test Plan

**Phase 2 STUB test (no SE, immediate):**
1. Flash Pico 2 with `VIGIA_PHASE2_STUB=1` build
2. Confirm Pi receives COBS frames at 1 Hz on `/dev/ttyACM0`
3. Confirm `SignedEt` published on `/vigia/signed_et` with `sig_valid=false`, `et_hash=[0×32]`, `ecdsa_sig=[0×64]`
4. Confirm `ImuSample` + `GpsPvt` still published from COBS packet fields (not text lines)
5. Confirm `SensorHealth` published: `parse_errors=0`, `imu_seq_gaps=0`

**Phase 2 LIVE test (after Ben wires SE):**
1. Flash Pico 2 with `VIGIA_PHASE2_STUB=0` build
2. Confirm `et_hash` is non-zero and consistent for same IMU/GPS input
3. Confirm `ecdsa_sig` is non-zero, 64 bytes
4. Enable mbedTLS verify on Pi — confirm `sig_valid=true`
5. Run 60-second soak: zero parse_errors, zero seq_gaps, `sig_valid=true` on every packet

---

## 8. Open Items / Decisions

| Item | Decision needed | Owner |
|---|---|---|
| Device public key distribution | Pre-provisioned in ATECC608A + PEM on Pi at `/etc/vigia/device_pubkey.pem` — who provisions? | Ben (hardware) + Tom (Pi setup) |
| Key slot assignment | Private key in slot 0; device ID in data zone — confirm with cryptoauthlib provisioning script | Ben |
| COBS backward compat cutover | Pi node auto-detects by first byte — no param needed | Tom (already in §4.1) |
| `SensorHealth` message | Needs `vigia_msgs/msg/SensorHealth.msg` created before colcon build | Tom |
| mbedTLS CMake integration | Add to `vigia_edge_node/CMakeLists.txt` before Pi build | Tom |
