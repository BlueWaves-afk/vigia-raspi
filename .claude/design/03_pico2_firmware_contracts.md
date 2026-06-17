# VIGIA ADAS DePIN Edge Node
## Raspberry Pi Pico 2 Firmware Contracts
**Document:** `03_pico2_firmware_contracts.md`  
**Depends on:** `01_system_architecture_and_roadmap.md` (APPROVED), `02_ros2_node_contracts.md` (APPROVED)  
**Status:** AWAITING APPROVAL — No implementation until sign-off  
**Scope:** Phase 2 — Raspberry Pi Pico 2 (RP2350) bare-metal firmware

---

## 0. Non-Negotiable Firmware Invariants

These rules are absolute. Any code violating them is non-conforming.

| Invariant | Enforcement |
|---|---|
| **No dynamic allocation** | `new`, `delete`, `malloc`, `free`, `std::vector`, `std::string` are **banned**. All buffers are `static` or stack-allocated with known, bounded sizes. `operator new` is overridden to `static_assert(false)` at link time. |
| **No RTOS** | FreeRTOS, ThreadX, Zephyr, and any scheduler are excluded. The execution model is an interrupt-driven DMA super-loop on **core 0 only**. Core 1 is parked. |
| **No blocking spin-wait in ISRs** | ISR handlers set atomic flags and return. All processing logic runs in the super-loop body, not inside interrupt context. The sole exception is the ATECC608A signing sequence (§6.3). |
| **No floating-point in ISR context** | ISRs never touch `float` or `double`. FPU data is only accessed in the super-loop. |
| **C++17 mandatory** | Firmware is compiled as C++17 (`-std=c++17`). Pico SDK C files are compiled as C11. |
| **Pico SDK only** | No STM32 HAL, no Arduino framework, no MicroPython in production firmware. |

---

## 1. MCU Specifications & Clock Tree

**Target board:** Raspberry Pi Pico 2  
**SoC:** RP2350  
**Core:** ARM Cortex-M33 @ 150 MHz (single-core firmware — core 1 parked at boot)  
**SRAM:** 520 KiB  
**Flash:** 4 MiB onboard QSPI  
**USB:** Native device controller via TinyUSB (Pico SDK)  
**Crystal:** 12 MHz onboard

### 1.1 Clock Configuration

```
System clock  = 150 MHz  (default Pico SDK `clock_configure`)
Peripheral clk = 150 MHz
SPI0 effective = 3.0 MHz  (BNO085 max — do not exceed)
UART1 baud     = 9600     (NEO-M8N default)
I2C1           = 400 kHz  (ATECC608A Fast Mode)
µs timer       = hardware_alarm or TIMER0 @ 1 MHz free-running 64-bit
```

> **SPI rate:** BNO085 SH-2 specification lists 3 MHz as maximum SCK. Higher rates are prohibited.

---

## 2. Pin Assignment Table

All pins are for the Raspberry Pi Pico 2 default header layout.

| Signal | GPIO | Direction | Function | Pull | Notes |
|---|---|---|---|---|---|
| **SPI0 — BNO085** |
| `SPI0_SCK` | GP18 | Out | SPI0 | None | 3.0 MHz, CPOL=0, CPHA=1 (Mode 1) |
| `SPI0_MOSI` | GP19 | Out | SPI0 | None | |
| `SPI0_MISO` | GP16 | In | SPI0 | None | |
| `BNO085_CSN` | GP17 | Out | GPIO | Pull-Up | Software CS — active LOW |
| `BNO085_INT` | GP20 | In | GPIO IRQ | Pull-Up | Active LOW, falling edge |
| `BNO085_RST` | GP21 | Out | GPIO | None | Active LOW reset |
| **UART1 — NEO-M8N GPS** |
| `UART1_TX` | GP8 | Out | UART1 | None | Reserved for GPS config |
| `UART1_RX` | GP9 | In | UART1 | Pull-Up | GPS UBX input |
| **I2C1 — ATECC608A** |
| `I2C1_SDA` | GP6 | In/Out | I2C1 | External 4.7 kΩ | Address 0x60 |
| `I2C1_SCL` | GP7 | Out | I2C1 | External 4.7 kΩ | 400 kHz |
| **USB — Pi 5 host** |
| USB D+/D− | — | — | TinyUSB CDC | — | Native USB device |
| **Diagnostics** |
| `LED_STATUS` | GP25 | Out | GPIO | None | Onboard LED |
| `LED_ERROR` | GP22 | Out | GPIO | None | External red LED |
| `HEARTBEAT_OUT` | GP26 | Out | GPIO | None | To Pi GPIO input |
| `PI_ALIVE_IN` | GP27 | In | GPIO | Pull-Up | From Pi GPIO output |

---

## 3. DMA Channel Assignment

RP2350 DMA channels allocated at init. No channel sharing between active peripherals.

| Peripheral | DMA Chan | Direction | Mode | Notes |
|---|---|---|---|---|
| SPI0 RX | 0 | Periph→Mem | Normal | BNO085 receive |
| SPI0 TX | 1 | Mem→Periph | Normal | BNO085 transmit (zeros) |
| UART1 RX | 2 | Periph→Mem | Circular | GPS ring buffer |

> Use `spi_set_format()`, `spi_init()`, and `dma_channel_configure()` from Pico SDK. UART RX uses `uart_set_irq_enables()` with IDLE detection via RX timeout or line-idle PIO fallback.

---

## 4. Static Memory Map

All buffers declared `static` at file scope. Zero heap usage.

```
SRAM Layout (520 KiB total):
┌─────────────────────────────────────────────────────┐
│  Pico SDK runtime + TinyUSB stack                    │  ~40 KB
├─────────────────────────────────────────────────────┤
│  .bss (static buffers)                               │  ~20 KB
│  ├─ BNO085 DMA RX buffer      [512 B]               │
│  ├─ BNO085 DMA TX buffer      [512 B]  (all zeros)  │
│  ├─ GPS UART ring buffer      [512 B]               │
│  ├─ GPS UBX frame buffer      [128 B]               │
│  ├─ E_t hash input buffer     [96 B]                │
│  ├─ COBS encode buffer        [300 B]               │
│  ├─ USB CDC TX buffer         [300 B]               │
│  ├─ cryptoauthlib cmd buf     [320 B]               │
│  └─ cryptoauthlib rsp buf     [192 B]               │
├─────────────────────────────────────────────────────┤
│  Main stack                                          │  8 KB
├─────────────────────────────────────────────────────┤
│  Available margin                                    │  ~450 KB
└─────────────────────────────────────────────────────┘

Total static footprint: ~68 KB / 520 KB (13%). Ample margin.
```

---

## 5. Execution Model: Interrupt-Driven DMA Super-Loop

The firmware has **no scheduler**. Core 0 runs a super-loop that sleeps with `__wfi()` and wakes on hardware interrupts. Core 1 is launched then immediately parked:

```cpp
// firmware/src/main.cpp
int main() {
    multicore_launch_core1(core1_park);  // core1: while(true) __wfi();
    vigia_main_loop();                  // never returns
}
```

```
                    ┌─────────────────────────────────────────┐
                    │              SUPER-LOOP (core 0)           │
  ┌──────────────── │  __wfi()  ← sleeps until any interrupt  │
  │                 └─────────────────────────────────────────┘
  │                                 │ wakes
  │                                 ▼
  │  ┌──────────────────────────────────────────────────────────┐
  │  │  Check deferred flags (set by ISRs, cleared here):        │
  │  │                                                            │
  │  │  if (g_bno085_frame_ready)  → Bno085Driver::process()    │
  │  │  if (g_gps_frame_ready)     → NeoM8nDriver::process()    │
  │  │                               → trigger sign+transmit    │
  │  │  tud_task()                 → TinyUSB CDC housekeeping    │
  │  └──────────────────────────────────────────────────────────┘
  │
  │  INTERRUPT SOURCES:
  │
  │  Priority highest: GPIO IRQ (BNO085 INT GP20)
  │  │  gpio_irq_handler()          → starts SPI0 DMA transfer
  │  │
  │  DMA IRQ: SPI0 RX complete
  │  │  dma_handler()               → sets g_bno085_frame_ready
  │  │
  │  UART1 IRQ: RX timeout / FIFO threshold
  │  │  uart_irq_handler()          → sets g_gps_frame_ready
  │  │
  │  hardware_alarm: µs tick overflow (64-bit timestamp)
  └─  alarm_irq()                    → increments timestamp high word
```

### 5.1 Atomic Flag Declarations

```cpp
// firmware/src/isr_flags.hpp
#pragma once
#include <atomic>
#include <cstdint>

inline std::atomic<bool> g_bno085_frame_ready{false};
inline std::atomic<bool> g_gps_frame_ready{false};
inline std::atomic<uint16_t> g_gps_bytes_received{0};
inline std::atomic<uint16_t> g_gps_ring_write_pos{0};
```

---

## 6. Component Drivers

Driver logic is **identical in behavior** to the prior STM32 spec. Only the HAL layer changes (Pico SDK instead of STM32 HAL). The wire protocol, struct layouts, and cryptographic contracts are unchanged.

---

### 6.1 `Bno085Driver` — SPI0 DMA (BNO085 IMU)

**File:** `firmware/src/bno085_driver.hpp` + `bno085_driver.cpp`

#### 6.1.1 Initialization Sequence

```
1. Assert BNO085_RST (GP21) LOW for 10 ms → release HIGH
2. Wait for BNO085_INT (GP20) falling edge
3. Read boot advertisement (SHTP channel 0) via SPI — discard
4. Send Set Feature: Rotation Vector (0x05) @ 10000 µs period
5. Send Set Feature: Linear Acceleration (0x04) @ 10000 µs period
6. Enable GPIO falling-edge IRQ on GP20
7. Super-loop begins
```

Set Feature payloads — same byte arrays as prior spec (report interval `0x00002710` LE = 10 ms).

#### 6.1.2 Class Definition

```cpp
class Bno085Driver {
public:
    struct Report {
        float    q_w, q_x, q_y, q_z;
        float    lin_accel_x, lin_accel_y, lin_accel_z;
        uint8_t  calibration_status;
        bool     valid;
    };

    void init(spi_inst_t* spi, uint cs_gpio, uint int_gpio, uint rst_gpio);
    void process();
    bool get_report(Report& out) const;

private:
    alignas(4) static uint8_t rx_buf_[512];
    alignas(4) static uint8_t tx_buf_[512];

    spi_inst_t* spi_{nullptr};
    uint cs_gpio_{0};

    static constexpr float kQuatScale  = 1.0f / 16384.0f;
    static constexpr float kAccelScale = 1.0f / 256.0f;

    volatile Report latest_report_{};
    std::atomic<bool> report_updated_{false};

    void parse_shtp_frame(const uint8_t* buf);
    void parse_rotation_vector(const uint8_t* payload);
    void parse_linear_accel(const uint8_t* payload);
};
```

#### 6.1.3 ISR → DMA → Super-Loop Flow

On GP20 falling edge: assert CS, start `spi_write_read_blocking` or DMA equivalent for 512 bytes, de-assert CS in DMA completion callback, set `g_bno085_frame_ready`. **No parsing in ISR.**

---

### 6.2 `NeoM8nDriver` — UART1 Ring Buffer (GPS)

**File:** `firmware/src/neo_m8n_driver.hpp` + `neo_m8n_driver.cpp`

UBX `NAV-PVT` (Class 0x01, ID 0x07) parsing — **unchanged** from prior spec.

```cpp
class NeoM8nDriver {
public:
    static constexpr uint16_t kRingBufSize = 512;
    static constexpr uint16_t kUbxFrameMax = 128;

    struct NavPvtReport {
        double   latitude, longitude;
        float    altitude_m, speed_ms, course_deg, hdop;
        uint8_t  fix_type, satellites;
        bool     valid;
    };

    void init(uart_inst_t* uart);
    void process();
    bool get_report(NavPvtReport& out) const;

private:
    alignas(4) static uint8_t ring_buf_[kRingBufSize];
    static uint8_t ubx_frame_buf_[kUbxFrameMax];
    uart_inst_t* uart_{nullptr};
    uint16_t last_read_pos_{0};
    // ... parse_nav_pvt(), validate_ubx_checksum() — same field offsets as prior spec
};
```

---

### 6.3 `Atecc608aDriver` — I2C1 Signing Pipeline (ATECC608A)

**File:** `firmware/src/atecc608a_driver.hpp` + `atecc608a_driver.cpp`

#### 6.3.1 cryptoauthlib HAL Integration

Use Microchip `cryptoauthlib` with the **RP2040 HAL** (`atca_hal_rp2040.c`). The RP2350 I2C API is compatible with the RP2040 HAL with minor clock defines.

```c
// firmware/third_party/cryptoauthlib/hal/atca_hal_rp2350.c
// Adapt from atca_hal_rp2040.c — i2c_write_blocking / i2c_read_blocking

ATCA_STATUS hal_i2c_send(void* iface, uint8_t* txdata, int txlength) {
    const int rc = i2c_write_blocking(I2C_PORT, ATECC608A_I2C_ADDR, txdata, txlength, false);
    return (rc == txlength) ? ATCA_SUCCESS : ATCA_COMM_FAIL;
}
```

Configure with `ATCA_NO_HEAP` in `atca_config.h`.

#### 6.3.2 E_t Payload — Cryptographic Contract (UNCHANGED)

```cpp
struct __attribute__((packed)) EtHashInput {
    uint8_t  device_id[16];
    uint64_t timestamp_us;
    uint32_t sequence;
    float    q_w, q_x, q_y, q_z;
    float    lin_accel_x, lin_accel_y, lin_accel_z;
    uint8_t  imu_cal_status;
    uint8_t  _pad0[3];
    double   latitude, longitude;
    float    altitude_m, speed_ms, course_deg;
    uint8_t  fix_type, satellites;
    uint8_t  _pad1[2];
    float    hdop;
};
static_assert(sizeof(EtHashInput) == 96, "EtHashInput layout changed — update Pi-side parser");
```

> **Critical:** Padding bytes MUST be zeroed before hashing. This struct is the ABI between Pico 2 firmware, Pi `SensorBridgeNode`, and cloud attestation server.

#### 6.3.3 Signing Sequence

```cpp
bool Atecc608aDriver::sign(const EtHashInput& payload,
                            uint8_t hash_out[32],
                            uint8_t sig_out[64]) const {
    ATCA_STATUS status = atcab_sha(sizeof(EtHashInput),
        reinterpret_cast<const uint8_t*>(&payload), hash_out);
    if (status != ATCA_SUCCESS) return false;
    status = atcab_sign(kKeySlot, hash_out, sig_out);
    return (status == ATCA_SUCCESS);
    // Total: ~57 ms (ATECC608A-bound, not MCU-bound)
}
```

---

### 6.4 `CobsTxDriver` — COBS Encoding & USB-CDC Transmission

**File:** `firmware/src/cobs_tx_driver.hpp` + `cobs_tx_driver.cpp`

#### 6.4.1 `SignedEtPacket` — Wire Struct (UNCHANGED)

```cpp
struct __attribute__((packed)) SignedEtPacket {
    uint8_t  version;           // 0x01
    uint8_t  type;              // 0x03 (SIGNED_ET)
    uint32_t sequence;
    uint64_t timestamp_us;      // Pico 2 µs timer at signing time
    float    q_w, q_x, q_y, q_z;
    float    lin_accel_x, lin_accel_y, lin_accel_z;
    uint8_t  imu_cal_status;
    double   latitude, longitude;
    float    altitude_m, speed_ms, course_deg;
    uint8_t  fix_type, satellites;
    uint8_t  _gps_pad[2];
    float    hdop;
    uint8_t  et_hash[32];
    uint8_t  ecdsa_sig[64];
};
static_assert(sizeof(SignedEtPacket) == 173, "SignedEtPacket size changed — update SensorBridgeNode parser");
```

#### 6.4.2 USB-CDC Transmission (TinyUSB)

```cpp
bool CobsTxDriver::transmit(const SignedEtPacket& pkt) {
    const size_t encoded_len = encode(
        reinterpret_cast<const uint8_t*>(&pkt), sizeof(SignedEtPacket),
        enc_buf_, kEncBufSize);

    if (!tud_cdc_connected()) return false;

    // Non-blocking: drop if TX FIFO full (acceptable at 10 Hz GPS rate)
    const uint32_t written = tud_cdc_write(enc_buf_, encoded_len);
    tud_cdc_write_flush();
    return written == encoded_len;
}
```

COBS encoder implementation — **identical** to prior spec (produces `[0x00][COBS data][0x00]`).

---

## 7. Main Loop Architecture

```cpp
extern "C" void vigia_main_loop() {
    board_init();
    stdio_init_all();          // TinyUSB CDC — disabled if using raw CDC only
    tusb_init();

    tim_init_microsecond_counter();
    bno085.init(spi0, GP17, GP20, GP21);
    gps.init(uart1);
    atecc.init();

    gpio_put(LED_STATUS, 1);
    uint32_t packet_sequence = 0;

    while (true) {
        __wfi();
        tud_task();

        if (g_bno085_frame_ready.load(std::memory_order_acquire)) {
            bno085.process();
        }

        if (g_gps_frame_ready.load(std::memory_order_acquire)) {
            g_gps_frame_ready.store(false, std::memory_order_relaxed);
            gps.process();

            Bno085Driver::Report imu{};
            NeoM8nDriver::NavPvtReport gps_report{};
            if (!bno085.get_report(imu) || !gps.get_report(gps_report)) continue;

            EtHashInput et{};
            // ... populate et from imu + gps (same field mapping as prior spec)
            et.timestamp_us = tim_get_microseconds();
            et.sequence = ++packet_sequence;
            std::memset(&et._pad0, 0, sizeof(et._pad0));
            std::memset(&et._pad1, 0, sizeof(et._pad1));

            uint8_t hash[32]{}, sig[64]{};
            if (!atecc.sign(et, hash, sig)) {
                gpio_put(LED_ERROR, 1);
                continue;
            }
            gpio_put(LED_ERROR, 0);

            SignedEtPacket pkt{};
            // ... assemble pkt (same mapping as prior spec)
            cobs_tx.transmit(pkt);

            if (packet_sequence % 10 == 0) gpio_xor_mask(1u << LED_STATUS);
        }
    }
}
```

---

## 8. `operator new` Deletion — Link-Time Enforcement

Same `no_heap.cpp` as prior spec — blocks all dynamic allocation at link time.

---

## 9. Firmware Build Configuration

### 9.1 `CMakeLists.txt` (Pico SDK)

```cmake
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
project(vigia_pico2 C CXX ASM)
pico_sdk_init()

add_executable(vigia_pico2
    src/main.cpp
    src/main_loop.cpp
    src/bno085_driver.cpp
    src/neo_m8n_driver.cpp
    src/atecc608a_driver.cpp
    src/cobs_tx_driver.cpp
    src/no_heap.cpp
    src/tim_us.cpp
    third_party/cryptoauthlib/hal/atca_hal_rp2350.c
    # ... cryptoauthlib sources (ATCA_NO_HEAP)
)

target_include_directories(vigia_pico2 PRIVATE
    src/
    third_party/cryptoauthlib/lib/
)

target_compile_definitions(vigia_pico2 PRIVATE
    ATCA_NO_HEAP
    ATCA_HAL_I2C
    ATCA_ATECC608A_SUPPORT
)

target_compile_options(vigia_pico2 PRIVATE
    -std=c++17
    -fno-exceptions
    -fno-rtti
    -Wall -Wextra
)

pico_enable_stdio_usb(vigia_pico2 0)   # raw TinyUSB CDC only
pico_add_extra_outputs(vigia_pico2)    # .uf2 + .bin
```

### 9.2 Flash Size Budget

| Region | Size | Notes |
|---|---|---|
| Pico SDK + TinyUSB | ~80 KB | |
| cryptoauthlib | ~40 KB | SHA + Sign only |
| Application C++ | ~30 KB | 4 driver classes + main loop |
| **Total Flash** | **~150 KB** | / 4096 KB available = 4% utilized |

### 9.3 Deploy

```bash
# Build
cmake -B build -DPICO_BOARD=pico2 && cmake --build build

# Flash (BOOTSEL + drag-and-drop)
cp build/vigia_pico2.uf2 /media/RP2350/

# Or via picotool
picotool load build/vigia_pico2.uf2 -f
```

---

## 10. IRQ Priority Table

| Source | Priority | Notes |
|---|---|---|
| GPIO BNO085 INT | Highest | Start SPI DMA immediately |
| DMA SPI0 complete | High | Set frame-ready flag |
| UART1 RX | Medium | GPS ring buffer drain signal |
| TinyUSB | Medium | CDC TX/RX — call `tud_task()` in super-loop |
| hardware_alarm (µs tick) | Low | Timestamp overflow |

---

## 11. File Structure

```
firmware/
├── CMakeLists.txt
├── pico_sdk_import.cmake
├── src/
│   ├── main.cpp                  # multicore park + vigia_main_loop()
│   ├── main_loop.cpp             # Super-loop orchestration
│   ├── bno085_driver.hpp/.cpp
│   ├── neo_m8n_driver.hpp/.cpp
│   ├── atecc608a_driver.hpp/.cpp
│   ├── cobs_tx_driver.hpp/.cpp
│   ├── tim_us.hpp/.cpp           # 64-bit µs counter
│   ├── isr_flags.hpp
│   └── no_heap.cpp
└── third_party/
    └── cryptoauthlib/
        ├── lib/
        └── hal/
            └── atca_hal_rp2350.c
```

---

## 12. Acceptance Criteria

| Test | Method | Pass Condition |
|---|---|---|
| **No dynamic allocation** | `nm build/vigia_pico2.elf \| grep ' malloc\b'` | Zero results |
| **No RTOS symbols** | `nm build/vigia_pico2.elf \| grep -E 'vTask\|osThread'` | Zero results |
| **Flash utilization** | `size build/vigia_pico2.elf` | `.text` ≤ 300 KB |
| **SRAM utilization** | `size build/vigia_pico2.elf` | `.bss` + `.data` ≤ 80 KB |
| **BNO085 data rate** | Logic analyzer on GP17 (CS) | ≥ 95 CS pulses/sec |
| **GPS parse rate** | UART monitor | ≥ 9 valid NAV-PVT frames/sec |
| **ECDSA sign latency** | µs timer delta: sign entry → CDC write | ≤ 75 ms |
| **COBS framing** | Pi-side `SensorBridgeNode` decode errors / 1000 packets | ≤ 1 error |
| **Anti-replay** | Replay same packet twice to Pi | Second packet dropped |
| **ECDSA verification** | Pi-side mbedTLS verify over 1000 packets | 0 failures |
| **USB enumeration** | `lsusb` on Pi | CDC device within 5s of power-on |

---

*Next document: `.claude/design/04_onnx_vision_engine_contracts.md` — ONNX Runtime + KleidiAI integration spec.*
