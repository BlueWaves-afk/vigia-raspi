# VIGIA ADAS DePIN Edge Node
## STM32F411 Black Pill Firmware Contracts
**Document:** `03_stm32_firmware_contracts.md`  
**Depends on:** `01_system_architecture_and_roadmap.md` (APPROVED), `02_ros2_node_contracts.md` (APPROVED)  
**Status:** AWAITING APPROVAL — No implementation until sign-off  
**Scope:** Phase 2 — STM32F411 Black Pill bare-metal firmware

---

## 0. Non-Negotiable Firmware Invariants

These rules are absolute. Any code violating them is non-conforming.

| Invariant | Enforcement |
|---|---|
| **No dynamic allocation** | `new`, `delete`, `malloc`, `free`, `std::vector`, `std::string` are **banned**. All buffers are `static` or stack-allocated with known, bounded sizes. `operator new` is overridden to `static_assert(false)` at link time. |
| **No RTOS** | FreeRTOS, ThreadX, Zephyr, and any scheduler are excluded. The execution model is an interrupt-driven DMA super-loop. |
| **No blocking spin-wait in ISRs** | ISR handlers set atomic flags and return. All processing logic runs in the super-loop body, not inside interrupt context. The sole exception is the ATECC608A signing sequence (§6.3). |
| **No floating-point in ISR context** | ISRs never touch `float` or `double`. The Cortex-M4F FPU context save/restore overhead (~200 ns) is unacceptable inside a DMA TC handler. FPU data is only accessed in the super-loop. |
| **C++17 mandatory** | Firmware is compiled as C++17 (`-std=c++17`). STM32 HAL C files are compiled as C11. |

---

## 1. MCU Specifications & Clock Tree

**Target MCU:** STM32F411CEU6 (WeAct Black Pill v3.1)  
**Core:** ARM Cortex-M4F @ 100 MHz  
**Flash:** 512 KB  
**SRAM:** 128 KB (single bank, no CCM)  
**USB:** OTG FS (PA11/PA12, internal PHY)  
**Crystal:** 25 MHz HSE

### 1.1 PLL Configuration (CubeMX `RCC_OscInitTypeDef`)

```
HSE = 25 MHz
PLL: M=25, N=200, P=2, Q=4
  → SYSCLK  = 25 × 200 / 2 = 100 MHz
  → USB CLK = 25 × 200 / 4 = 50 MHz  ← USB OTG FS requires exactly 48 MHz
```

**USB clock correction:** USB OTG FS requires exactly 48 MHz on the PLL48CK output.

```
Corrected PLL: M=25, N=192, P=2, Q=4
  → SYSCLK  = 25 × 192 / 2 = 96 MHz   (within STM32F411 100 MHz max)
  → USB CLK = 25 × 192 / 4 = 48 MHz   ← exact requirement met
```

**All subsequent peripheral clock derivations use SYSCLK = 96 MHz.**

| Bus | Prescaler | Clock | Max Allowed |
|---|---|---|---|
| AHB1 (DMA, GPIO) | /1 | 96 MHz | 100 MHz ✓ |
| APB1 (USART2, I2C1) | /2 | 48 MHz | 50 MHz ✓ |
| APB2 (SPI1, USART1) | /1 | 96 MHz | 100 MHz ✓ |
| SPI1 effective rate | APB2/32 | **3.0 MHz** | BNO085 max 3 MHz ✓ |
| USART2 baud | APB1 OVER16 | **9600 baud** | NEO-M8N default ✓ |
| I2C1 CCR | APB1/40 | **400 kHz** | ATECC608A 1 MHz max ✓ |
| TIM2 (µs tick) | APB1×2/48 | **1 MHz** | Free-running 32-bit µs counter |

> **Note on SPI rate:** The BNO085 SH-2 interface specification lists 3 MHz as the maximum SCK frequency. The user specification targets "4 MHz DMA bandwidth" — the 3.0 MHz achieved with APB2/32 is the closest standard prescaler that stays within the BNO085's guaranteed maximum. APB2/16 = 6 MHz exceeds the BNO085 limit and is prohibited.

---

## 2. Pin Assignment Table

All pins are for the WeAct STM32F411 Black Pill v3.1 (48-pin LQFP).

| Signal | Pin | Direction | AF / Mode | Pull | Notes |
|---|---|---|---|---|---|
| **SPI1 — BNO085** |
| `SPI1_SCK` | PA5 | Out | AF5 | None | 3.0 MHz, CPOL=0, CPHA=1 (SPI Mode 1 — BNO085 requirement) |
| `SPI1_MISO` | PA6 | In | AF5 | None | BNO085 MISO |
| `SPI1_MOSI` | PA7 | Out | AF5 | None | BNO085 MOSI |
| `BNO085_CSN` | PA4 | Out | GPIO Output | Pull-Up | Software CS — active LOW. **Not AF** — manually toggled. |
| `BNO085_INT` | PB0 | In | GPIO EXTI0 | Pull-Up | BNO085 data-ready — active LOW, falling edge EXTI |
| `BNO085_RST` | PB1 | Out | GPIO Output | None | BNO085 reset — active LOW |
| **USART2 — NEO-M8N GPS** |
| `USART2_TX` | PA2 | Out | AF7 | None | Pi→GPS not needed; TX reserved for future config |
| `USART2_RX` | PA3 | In | AF7 | Pull-Up | GPS NMEA/UBX input |
| **I2C1 — ATECC608A** |
| `I2C1_SCL` | PB6 | Out | AF4 (OD) | External 4.7 kΩ | 400 kHz Fast Mode |
| `I2C1_SDA` | PB7 | In/Out | AF4 (OD) | External 4.7 kΩ | ATECC608A I2C address: 0x60 |
| **USB OTG FS — Pi 5 host** |
| `USB_DM` | PA11 | In/Out | AF10 | None | Fixed on STM32F411 |
| `USB_DP` | PA12 | In/Out | AF10 | None | Fixed on STM32F411 |
| **Diagnostics** |
| `LED_STATUS` | PC13 | Out | GPIO Output | None | Active LOW (Black Pill on-board LED) |
| `LED_ERROR` | PB12 | Out | GPIO Output | None | External red LED — error/fault indicator |
| **Reserved / Future** |
| `UPS_WAKE` | PA8 | In | GPIO EXTI8 | Pull-Up | Reserved for future UPS alert (not used Phase 2) |

---

## 3. DMA Stream / Channel Assignment

STM32F411 has DMA1 (7 streams) and DMA2 (8 streams). Assignments must not conflict.

| Peripheral | DMA | Stream | Channel | Direction | Mode | Notes |
|---|---|---|---|---|---|---|
| SPI1 RX | DMA2 | Stream 0 | Channel 3 | Periph→Mem | Normal | BNO085 receive |
| SPI1 TX | DMA2 | Stream 3 | Channel 3 | Mem→Periph | Normal | BNO085 transmit (zeros) |
| USART2 RX | DMA1 | Stream 5 | Channel 4 | Periph→Mem | **Circular** | GPS ring buffer |
| USART2 TX | DMA1 | Stream 6 | Channel 4 | Mem→Periph | Normal | Reserved (unused Phase 2) |

> **Conflict check:** No two entries share the same DMA controller + Stream combination. DMA2 Stream 0 and Stream 3 are on separate streams. DMA1 Stream 5 and Stream 6 are on separate streams. No conflicts.

---

## 4. Static Memory Map

All buffers declared `static` at file scope (BSS or data segment). Zero heap usage.

```
SRAM Layout (128 KB total):
┌─────────────────────────────────────────────────────┐  0x20000000
│  Interrupt Vector Table / System                     │  ~1 KB
├─────────────────────────────────────────────────────┤
│  .data (initialized globals)                         │  ~4 KB
├─────────────────────────────────────────────────────┤
│  .bss (zero-initialized globals / static buffers)   │  ~20 KB
│  ├─ BNO085 DMA RX buffer      [512 B]               │
│  ├─ BNO085 DMA TX buffer      [512 B]  (all zeros)  │
│  ├─ GPS USART DMA ring buffer [512 B]               │
│  ├─ GPS UBX frame buffer      [128 B]               │
│  ├─ E_t hash input buffer     [96 B]                │
│  ├─ COBS encode buffer        [300 B]               │
│  ├─ USB CDC TX buffer         [300 B]               │
│  ├─ cryptoauthlib cmd buf     [320 B]  (ATCA_CMD_SIZE_MIN) │
│  └─ cryptoauthlib rsp buf     [192 B]               │
├─────────────────────────────────────────────────────┤
│  USB middleware buffers (ST USB stack)               │  ~8 KB
├─────────────────────────────────────────────────────┤
│  Main stack (MSP)                                    │  8 KB
├─────────────────────────────────────────────────────┤
│  Available margin                                    │  ~87 KB
└─────────────────────────────────────────────────────┘  0x20020000

Total static footprint: ~41 KB / 128 KB (32%). Ample margin.
```

---

## 5. Execution Model: Interrupt-Driven DMA Super-Loop

The firmware has **no scheduler**. The main loop sleeps with `__WFI()` and wakes strictly on hardware interrupts. Interrupt Service Routines set `volatile` atomic state flags; all non-trivial processing runs in the super-loop body after `__WFI()` returns.

```
                    ┌─────────────────────────────────────────┐
                    │              SUPER-LOOP                  │
  ┌──────────────── │  __WFI()  ← sleeps until any interrupt  │
  │                 └─────────────────────────────────────────┘
  │                                 │ wakes
  │                                 ▼
  │  ┌──────────────────────────────────────────────────────────┐
  │  │  Check deferred flags (set by ISRs, cleared here):        │
  │  │                                                            │
  │  │  if (g_bno085_frame_ready)  → Bno085Driver::process()    │
  │  │  if (g_gps_frame_ready)     → NeoM8nDriver::process()    │
  │  │                               → trigger sign+transmit    │
  │  └──────────────────────────────────────────────────────────┘
  │
  │  INTERRUPT SOURCES (in priority order, Cortex-M4 NVIC):
  │
  │  IRQ Priority 1 (highest): USB OTG FS
  │  │  OTG_FS_IRQHandler()         — ST USB stack, handles CDC TX/RX
  │  │
  │  IRQ Priority 2: EXTI0 (BNO085 INT pin PB0)
  │  │  EXTI0_IRQHandler()          → starts SPI1 DMA transfer
  │  │
  │  IRQ Priority 3: DMA2 Stream0 (SPI1 RX complete)
  │  │  DMA2_Stream0_IRQHandler()   → sets g_bno085_frame_ready
  │  │
  │  IRQ Priority 3: DMA2 Stream3 (SPI1 TX complete — housekeeping only)
  │  │  DMA2_Stream3_IRQHandler()   → clears CS line, asserts idle
  │  │
  │  IRQ Priority 4: USART2 (IDLE line detection)
  │  │  USART2_IRQHandler()         → computes bytes received, sets g_gps_frame_ready
  │  │
  │  IRQ Priority 5 (lowest): TIM2 overflow (µs tick — 32-bit free-running)
  └─    TIM2_IRQHandler()           → increments high 32 bits of 64-bit µs timestamp
```

### 5.1 Atomic Flag Declarations

All cross-ISR/super-loop flags use `volatile` + `std::atomic`. No `std::mutex` — mutexes require RTOS primitives that don't exist here.

```cpp
// firmware/src/isr_flags.hpp
#pragma once
#include <atomic>
#include <cstdint>

// Set by DMA2 Stream0 TC ISR; cleared by super-loop after Bno085Driver::process()
inline std::atomic<bool> g_bno085_frame_ready{false};

// Set by USART2 IDLE ISR; cleared by super-loop after NeoM8nDriver::process()
inline std::atomic<bool> g_gps_frame_ready{false};

// Number of bytes received in last USART2 DMA burst (set by ISR, read by super-loop)
inline std::atomic<uint16_t> g_gps_bytes_received{0};

// Current write index in GPS DMA ring buffer (set by ISR)
inline std::atomic<uint16_t> g_gps_dma_write_pos{0};
```

---

## 6. Component Drivers

---

### 6.1 `Bno085Driver` — SPI1 DMA (BNO085 IMU)

**File:** `firmware/src/bno085_driver.hpp` + `bno085_driver.cpp`

#### 6.1.1 BNO085 SHTP Protocol Summary

The BNO085 uses Hillcrest's Sensor Hub Transport Protocol (SHTP) over SPI. The host is always SPI master. The BNO085 asserts `INT` (active LOW) when it has data to send. Every SPI transaction begins with a 4-byte SHTP header, followed by the payload.

```
SHTP Frame on wire:
  [Byte 0] Length LSB      }
  [Byte 1] Length MSB (b7=continuation bit, b6:0=high length bits) }  Header (4 bytes)
  [Byte 2] Channel ID     }  Channel 3 = Sensor Reports
  [Byte 3] Sequence Number}
  [Byte 4..N] Payload
```

The `Length` field includes the 4 header bytes. Maximum useful SHTP payload: 256 bytes. Total max frame: 260 bytes. Buffer allocation: **512 bytes** (2× margin, DMA alignment).

#### 6.1.2 BNO085 Initialization Sequence (called once from `main()`)

```
1. Assert BNO085_RST (PB1) LOW for 10 ms → release HIGH
2. Wait for BNO085_INT (PB0) falling edge (boot advertisement on channel 0)
3. Read boot advertisement (SHTP channel 0 frame) via SPI — discard
4. Send "Set Feature Command" (report_id=0xFD) on channel 2:
   Enable Rotation Vector (report 0x05) at 10 ms period (100 Hz)
5. Send "Set Feature Command" on channel 2:
   Enable Linear Acceleration (report 0x04) at 10 ms period (100 Hz)
6. Enable EXTI0 falling-edge interrupt on PB0
7. Super-loop begins
```

Set Feature Command payload (17 bytes):
```cpp
static constexpr uint8_t kSetFeatureRotVec[17] = {
    0xFD,           // Set Feature Command
    0x05,           // Report ID: Rotation Vector
    0x00, 0x00,     // Feature flags (none)
    0x00, 0x00,     // Change sensitivity (disabled)
    0x10, 0x27, 0x00, 0x00,  // Report interval: 10000 µs = 0x00002710 LE
    0x00, 0x00, 0x00, 0x00,  // Batch interval (disabled)
    0x00, 0x00, 0x00         // Sensor-specific config (unused)
};

static constexpr uint8_t kSetFeatureLinAccel[17] = {
    0xFD, 0x04,     // Linear Acceleration report
    0x00, 0x00, 0x00, 0x00,
    0x10, 0x27, 0x00, 0x00,  // 10000 µs
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00
};
```

#### 6.1.3 ISR → DMA → Super-Loop Flow

```
PB0 FALLING EDGE (BNO085 data ready)
         │
         ▼
EXTI0_IRQHandler():
  if (SPI1 not busy):
      GPIO_WritePin(BNO085_CSN, LOW)       // assert CS
      HAL_SPI_TransmitReceive_DMA(
          &hspi1,
          tx_buf_,           // pre-zeroed 512B static buffer
          rx_buf_,           // 512B static DMA-aligned buffer
          512                // always read max frame size
      )
  // return — DO NOT process data here

         │
         ▼ (DMA2 Stream0 + Stream3 complete simultaneously)

DMA2_Stream0_IRQHandler() [RX complete — this is the one we care about]:
  GPIO_WritePin(BNO085_CSN, HIGH)          // de-assert CS
  HAL_DMA_IRQHandler(&hdma_spi1_rx)        // clear DMA flags
  g_bno085_frame_ready.store(true, std::memory_order_release)
  // return — NO data parsing in ISR

DMA2_Stream3_IRQHandler() [TX complete — housekeeping]:
  HAL_DMA_IRQHandler(&hdma_spi1_tx)
  // return — nothing else needed

         │
         ▼ (super-loop wakes from __WFI)

Bno085Driver::process():  // runs in super-loop context, FPU safe
  if (!g_bno085_frame_ready.load(std::memory_order_acquire)) return;
  g_bno085_frame_ready.store(false, std::memory_order_relaxed);
  parse_shtp_frame(rx_buf_);               // parse SHTP header + payload
```

#### 6.1.4 Class Definition

```cpp
// firmware/src/bno085_driver.hpp
#pragma once
#include <cstdint>
#include <atomic>
#include "stm32f4xx_hal.h"

class Bno085Driver {
public:
    // Parsed output — updated by process(), read by super-loop
    struct Report {
        float    q_w, q_x, q_y, q_z;        // Unit quaternion (body→world)
        float    lin_accel_x;                 // Body-frame linear accel m/s²
        float    lin_accel_y;
        float    lin_accel_z;
        uint8_t  calibration_status;          // 0=uncal, 3=fully calibrated
        bool     valid;                       // false until first report received
    };

    void init(SPI_HandleTypeDef* hspi);
    void process();                           // call from super-loop only
    bool get_report(Report& out) const;       // returns false if no new data since last call

private:
    // DMA buffers — 4-byte aligned for DMA controller requirement
    alignas(4) static uint8_t rx_buf_[512];
    alignas(4) static uint8_t tx_buf_[512];  // permanently zeroed — BNO085 reads during TX phase

    SPI_HandleTypeDef* hspi_{nullptr};

    // Internal parse state
    enum class ParseState : uint8_t { IDLE, AWAITING_REPORT };
    ParseState parse_state_{ParseState::IDLE};

    // Q-point fixed-point scale factors (BNO085 SH-2 spec §6.5.18)
    static constexpr float kQuatScale    = 1.0f / 16384.0f;   // Q14
    static constexpr float kAccelScale   = 1.0f / 256.0f;     // Q8, units: m/s²

    volatile Report latest_report_{};
    std::atomic<bool> report_updated_{false};

    void parse_shtp_frame(const uint8_t* buf);
    void parse_rotation_vector(const uint8_t* payload);
    void parse_linear_accel(const uint8_t* payload);
};
```

#### 6.1.5 Fixed-Point → Float Conversion

BNO085 reports use Q-point fixed-point integers. Conversion in `parse_rotation_vector()`:

```cpp
void Bno085Driver::parse_rotation_vector(const uint8_t* payload) {
    // payload[0] = Report ID (0x05) — already verified by caller
    // payload[1] = Sequence number
    // payload[2] = Status / calibration accuracy
    // payload[3] = Delay (ignored)
    // payload[4..5] = i (Q14 int16)
    // payload[6..7] = j (Q14 int16)
    // payload[8..9] = k (Q14 int16)
    // payload[10..11] = real (Q14 int16)
    // payload[12..13] = accuracy estimate (Q12, degrees — not used)

    auto read_i16 = [](const uint8_t* p) -> int16_t {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
    };

    // BNO085 reports quaternion as (i, j, k, real) — reorder to (w, x, y, z)
    const float qi = read_i16(payload + 4)  * kQuatScale;
    const float qj = read_i16(payload + 6)  * kQuatScale;
    const float qk = read_i16(payload + 8)  * kQuatScale;
    const float qr = read_i16(payload + 10) * kQuatScale;

    latest_report_.q_w = qr;
    latest_report_.q_x = qi;
    latest_report_.q_y = qj;
    latest_report_.q_z = qk;
    latest_report_.calibration_status = payload[2] & 0x03;  // low 2 bits = accuracy
    latest_report_.valid = true;
    report_updated_.store(true, std::memory_order_release);
}
```

---

### 6.2 `NeoM8nDriver` — USART2 DMA Circular Ring Buffer (GPS)

**File:** `firmware/src/neo_m8n_driver.hpp` + `neo_m8n_driver.cpp`

#### 6.2.1 UBX Binary Protocol Summary

The NEO-M8N outputs UBX binary frames. We parse the `NAV-PVT` message (Class 0x01, ID 0x07) which contains all required navigation data in a single 100-byte frame.

```
UBX Frame structure:
  [0xB5][0x62]           — sync chars (2 bytes)
  [CLASS][ID]            — 0x01, 0x07 for NAV-PVT
  [Length LSB][Length MSB] — payload length (92 for NAV-PVT)
  [Payload: 92 bytes]
  [CK_A][CK_B]           — Fletcher-8 checksum over CLASS..Payload
  Total: 100 bytes
```

The DMA circular mode continuously fills a 512-byte ring buffer. USART2's IDLE line interrupt fires when the GPS stops transmitting (end of UBX frame). The ISR computes how many new bytes arrived by comparing the current DMA NDTR register to the last known position.

#### 6.2.2 DMA Circular Mode Configuration

```cpp
// DMA1 Stream5 Channel4 — USART2 RX — Circular mode
// HAL_UART_Receive_DMA() is called ONCE at init; DMA runs forever.
// Never call HAL_UART_Receive_DMA() again — it would reset the ring pointer.

void NeoM8nDriver::init(UART_HandleTypeDef* huart) {
    huart_ = huart;
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);          // enable IDLE interrupt
    HAL_UART_Receive_DMA(huart_, dma_ring_buf_, kRingBufSize);  // start circular DMA
}
```

Circular DMA means: when the DMA reaches the end of `dma_ring_buf_[512]`, it wraps to `[0]` automatically. The DMA NDTR register counts DOWN from 512. The ISR reconstructs the write position as `kRingBufSize - DMA_GetCurrDataCounter(...)`.

#### 6.2.3 USART2 IDLE Interrupt Handler

```cpp
// Called from USART2_IRQHandler() — in stm32f4xx_it.cpp
extern "C" void USART2_IRQHandler() {
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        // Compute new write position from DMA NDTR
        const uint16_t dma_remaining =
            static_cast<uint16_t>(__HAL_DMA_GET_COUNTER(huart2.hdmarx));
        const uint16_t write_pos =
            static_cast<uint16_t>(NeoM8nDriver::kRingBufSize - dma_remaining);

        g_gps_dma_write_pos.store(write_pos, std::memory_order_release);
        g_gps_frame_ready.store(true, std::memory_order_release);
        // DO NOT process frame here — return immediately
    }
    HAL_UART_IRQHandler(&huart2);
}
```

#### 6.2.4 Class Definition

```cpp
// firmware/src/neo_m8n_driver.hpp
#pragma once
#include <cstdint>
#include <atomic>
#include "stm32f4xx_hal.h"

class NeoM8nDriver {
public:
    static constexpr uint16_t kRingBufSize  = 512;
    static constexpr uint16_t kUbxFrameMax  = 128;

    struct NavPvtReport {
        double   latitude;          // degrees (WGS-84), 1e-7 deg resolution
        double   longitude;         // degrees (WGS-84)
        float    altitude_m;        // height above ellipsoid, mm→m
        float    speed_ms;          // ground speed, mm/s→m/s
        float    course_deg;        // heading, 1e-5 deg→deg
        uint8_t  fix_type;          // 0=none 1=DR 2=2D 3=3D 4=GNSS+DR
        float    hdop;              // horizontal DOP (scaled ×0.01)
        uint8_t  satellites;
        bool     valid;
    };

    void init(UART_HandleTypeDef* huart);
    void process();                // call from super-loop when g_gps_frame_ready
    bool get_report(NavPvtReport& out) const;

private:
    alignas(4) static uint8_t dma_ring_buf_[kRingBufSize];  // DMA destination — must not be touched outside ISR/DMA
    static uint8_t ubx_frame_buf_[kUbxFrameMax];            // assembled UBX frame

    UART_HandleTypeDef* huart_{nullptr};
    uint16_t last_read_pos_{0};   // last position we consumed from ring buffer

    volatile NavPvtReport latest_report_{};
    std::atomic<bool> report_updated_{false};

    // Returns number of bytes copied into ubx_frame_buf_ (0 if no complete frame found)
    uint16_t drain_ring_buffer(uint16_t new_write_pos);

    // Returns true if UBX checksum passes
    bool validate_ubx_checksum(const uint8_t* frame, uint16_t len) const;

    void parse_nav_pvt(const uint8_t* payload);
};
```

#### 6.2.5 UBX Ring Buffer Drain Logic

```
Super-loop calls process() when g_gps_frame_ready is true:

1. Read g_gps_dma_write_pos (set by ISR) → new_write_pos
2. Compute bytes_available = (new_write_pos - last_read_pos_) % kRingBufSize
3. Copy bytes_available bytes from ring buffer into ubx_frame_buf_,
   handling wrap-around at ring end
4. Update last_read_pos_
5. Scan ubx_frame_buf_ for [0xB5 0x62 0x01 0x07] sync + class/id pattern
6. Verify length field == 92 (NAV-PVT payload size)
7. Validate Fletcher-8 checksum (CK_A, CK_B over CLASS..Payload)
8. If valid: parse_nav_pvt() → update latest_report_
9. If invalid: discard and reset scan position (GPS may have sent partial frame)
```

#### 6.2.6 `NAV-PVT` Payload Field Extraction

```cpp
void NeoM8nDriver::parse_nav_pvt(const uint8_t* payload) {
    // NAV-PVT payload offsets (UBX Protocol Specification §32.17.14.1):
    // [0..3]  iTOW (ms, GPS time of week) — ignored
    // [4..5]  year, [6] month, [7] day — ignored
    // [8..11] nano (ns) — ignored
    // [20]    fixType
    // [21]    flags (bit 0 = gnssFixOK)
    // [23]    numSV (satellites used)
    // [24..27] lon (deg × 1e-7, int32)
    // [28..31] lat (deg × 1e-7, int32)
    // [32..35] height (mm, int32) — above ellipsoid
    // [60..63] gSpeed (mm/s, int32) — ground speed
    // [64..67] headMot (deg × 1e-5, int32) — heading of motion
    // [76..77] pDOP (× 0.01, uint16)

    auto read_i32 = [](const uint8_t* p) -> int32_t {
        return static_cast<int32_t>(
            static_cast<uint32_t>(p[0])       |
            (static_cast<uint32_t>(p[1]) << 8) |
            (static_cast<uint32_t>(p[2]) << 16)|
            (static_cast<uint32_t>(p[3]) << 24));
    };
    auto read_u16 = [](const uint8_t* p) -> uint16_t {
        return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    };

    latest_report_.longitude  = read_i32(payload + 24) * 1e-7;
    latest_report_.latitude   = read_i32(payload + 28) * 1e-7;
    latest_report_.altitude_m = read_i32(payload + 32) * 0.001f;   // mm → m
    latest_report_.speed_ms   = read_i32(payload + 60) * 0.001f;   // mm/s → m/s
    latest_report_.course_deg = read_i32(payload + 64) * 1e-5f;    // 1e-5 deg → deg
    latest_report_.fix_type   = payload[20];
    latest_report_.hdop       = read_u16(payload + 76) * 0.01f;
    latest_report_.satellites = payload[23];
    latest_report_.valid      = (payload[21] & 0x01) != 0;          // gnssFixOK bit
    report_updated_.store(true, std::memory_order_release);
}
```

---

### 6.3 `Atecc608aDriver` — I2C Signing Pipeline (ATECC608A)

**File:** `firmware/src/atecc608a_driver.hpp` + `atecc608a_driver.cpp`

#### 6.3.1 cryptoauthlib HAL Integration

The Microchip `cryptoauthlib` library requires a platform HAL layer to be provided. For STM32 HAL, we implement the four required function stubs. cryptoauthlib is configured with `ATCA_NO_HEAP` (in `atca_config.h`) to eliminate its internal `malloc()` usage. All cryptoauthlib command/response buffers become statically allocated.

```c
// firmware/third_party/cryptoauthlib/hal/atca_hal_stm32.c
// These 4 functions are the complete HAL implementation:

ATCA_STATUS hal_i2c_init(void* hal, ATCAIfaceCfg* cfg) {
    // cfg->atcai2c.slave_address = 0xC0 (0x60 << 1, 7-bit addr 0x60)
    // cfg->atcai2c.bus = 1 (I2C1)
    // cfg->atcai2c.baud = 400000
    return ATCA_SUCCESS;  // I2C1 already initialized by CubeMX-generated code
}

ATCA_STATUS hal_i2c_send(void* iface, uint8_t* txdata, int txlength) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
        &hi2c1,
        ATECC608A_I2C_ADDR << 1,    // 7-bit addr shifted for HAL
        txdata, txlength,
        HAL_MAX_DELAY);
    return (status == HAL_OK) ? ATCA_SUCCESS : ATCA_COMM_FAIL;
}

ATCA_STATUS hal_i2c_receive(void* iface, uint8_t* rxdata, uint16_t* rxlength) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
        &hi2c1,
        ATECC608A_I2C_ADDR << 1,
        rxdata, *rxlength,
        HAL_MAX_DELAY);
    return (status == HAL_OK) ? ATCA_SUCCESS : ATCA_COMM_FAIL;
}

ATCA_STATUS hal_i2c_wake(void* iface)  { /* ATECC608A wake sequence */ ... }
ATCA_STATUS hal_i2c_idle(void* iface)  { atcab_idle(); return ATCA_SUCCESS; }
ATCA_STATUS hal_i2c_sleep(void* iface) { atcab_sleep(); return ATCA_SUCCESS; }
```

#### 6.3.2 E_t Payload Construction and Signing

The E_t payload is the data over which SHA-256 is computed and ECDSA signing is performed. It is a fixed-layout binary struct — packed for deterministic serialization.

```cpp
// firmware/src/atecc608a_driver.hpp

// E_t hash input — the "kinematic context" struct.
// This exact byte layout is what gets SHA-256 hashed.
// Both ends (STM32 and Pi) must agree on this layout.
struct __attribute__((packed)) EtHashInput {
    uint8_t  device_id[16];         // UUID provisioned in ATECC608A User Zone 0
    uint64_t timestamp_us;           // TIM2 free-running µs counter (64-bit)
    uint32_t sequence;               // Monotonic packet counter
    // IMU fields (from Bno085Driver::Report)
    float    q_w, q_x, q_y, q_z;   // Body→world quaternion
    float    lin_accel_x;
    float    lin_accel_y;
    float    lin_accel_z;
    uint8_t  imu_cal_status;
    uint8_t  _pad0[3];              // Explicit padding — never leave implicit padding in hashed structs
    // GPS fields (from NeoM8nDriver::NavPvtReport)
    double   latitude;
    double   longitude;
    float    altitude_m;
    float    speed_ms;
    float    course_deg;
    uint8_t  fix_type;
    uint8_t  satellites;
    uint8_t  _pad1[2];
    float    hdop;
};
static_assert(sizeof(EtHashInput) == 96, "EtHashInput layout changed — update Pi-side parser");
```

> **Critical:** All padding bytes (`_pad0`, `_pad1`) MUST be explicitly zeroed before hashing. Never leave implicit compiler padding in a struct that gets hashed — different compiler versions may initialize padding differently, causing hash mismatches between the STM32 and Pi-side ECDSA verifier.

#### 6.3.3 Signing Sequence

```cpp
// firmware/src/atecc608a_driver.hpp

class Atecc608aDriver {
public:
    static constexpr uint8_t kKeySlot = 0;  // ECC private key provisioned in slot 0

    bool init();

    // Full sign sequence — blocking (called from super-loop, ~60ms total)
    // Returns false on any cryptoauthlib error.
    bool sign(const EtHashInput& payload,
              uint8_t hash_out[32],          // SHA-256 output
              uint8_t sig_out[64]) const;    // ECDSA secp256r1 raw (R,S) output

private:
    // Static cryptoauthlib buffers (ATCA_NO_HEAP must be defined in atca_config.h)
    static uint8_t atca_cmd_buf_[ATCA_CMD_SIZE_MAX];   // 320 bytes
    static uint8_t atca_rsp_buf_[ATCA_RSP_SIZE_MAX];   // 192 bytes

    ATCAIfaceCfg cfg_{};
    bool initialized_{false};
};
```

```cpp
// firmware/src/atecc608a_driver.cpp

bool Atecc608aDriver::sign(const EtHashInput& payload,
                            uint8_t hash_out[32],
                            uint8_t sig_out[64]) const {
    if (!initialized_) return false;

    // Step 1: SHA-256 via ATECC608A internal engine
    // atcab_sha() streams data in 64-byte blocks; handles all chunking internally
    ATCA_STATUS status = atcab_sha(
        sizeof(EtHashInput),
        reinterpret_cast<const uint8_t*>(&payload),
        hash_out);
    if (status != ATCA_SUCCESS) return false;

    // Step 2: ECDSA sign — private key in slot kKeySlot (never leaves the chip)
    // hash_out is the 32-byte SHA-256 digest computed above
    // sig_out receives 64-byte raw IEEE P1363 signature (r ∥ s, 32 bytes each)
    status = atcab_sign(kKeySlot, hash_out, sig_out);
    return (status == ATCA_SUCCESS);

    // Total wall-clock time:
    //   atcab_sha()  on 96 bytes:  ~3 I2C transactions ≈ 5 ms
    //   atcab_sign() execution:    ~50 ms (ATECC608A ECDSA execution time)
    //   I2C overhead:              ~2 ms
    //   Total:                     ~57 ms @ 400 kHz I2C
}
```

---

### 6.4 `CobsTxDriver` — COBS Encoding & USB-CDC Transmission

**File:** `firmware/src/cobs_tx_driver.hpp` + `cobs_tx_driver.cpp`

#### 6.4.1 `SignedEtPacket` C++ Struct

This is the complete wire packet transmitted to the Pi over USB-CDC. All fields are little-endian (ARM native). The Pi-side `SensorBridgeNode` parses this struct by direct memory mapping after COBS decode.

```cpp
// firmware/src/cobs_tx_driver.hpp

struct __attribute__((packed)) SignedEtPacket {
    // ── Frame header ──────────────────────────────────── 14 bytes
    uint8_t  version;           // Protocol version = 0x01
    uint8_t  type;              // Packet type = 0x03 (SIGNED_ET)
    uint32_t sequence;          // Monotonic counter (anti-replay)
    uint64_t timestamp_us;      // STM32 TIM2 µs timestamp at signing time

    // ── IMU data ──────────────────────────────────────── 29 bytes
    float    q_w, q_x, q_y, q_z;           // Unit quaternion
    float    lin_accel_x, lin_accel_y, lin_accel_z;  // m/s²
    uint8_t  imu_cal_status;

    // ── GPS data ──────────────────────────────────────── 34 bytes
    double   latitude;          // degrees (8 bytes)
    double   longitude;         // degrees (8 bytes)
    float    altitude_m;        // 4 bytes
    float    speed_ms;          // 4 bytes
    float    course_deg;        // 4 bytes
    uint8_t  fix_type;          // 1 byte
    uint8_t  satellites;        // 1 byte
    uint8_t  _gps_pad[2];       // explicit padding
    float    hdop;              // 4 bytes

    // ── Cryptographic fields ──────────────────────────── 96 bytes
    uint8_t  et_hash[32];       // SHA-256(EtHashInput)
    uint8_t  ecdsa_sig[64];     // secp256r1 raw R∥S signature

    // ── Total ─────────────────────────────────────────── 173 bytes
};
static_assert(sizeof(SignedEtPacket) == 173, "SignedEtPacket size changed — update SensorBridgeNode parser");
```

**Device certificate is NOT transmitted in the packet.** The Pi side loads the device X.509 certificate from `/etc/vigia/device_cert.pem` at `SensorBridgeNode` startup and holds it in memory for ECDSA verification. Transmitting a DER certificate (~1–2 KB) per packet at 10 Hz would consume ~160 KB/s of the USB-CDC bandwidth unnecessarily.

#### 6.4.2 COBS Encoder

```cpp
// firmware/src/cobs_tx_driver.hpp — static, no-alloc COBS encoder

class CobsTxDriver {
public:
    // Maximum encoded output size for a 173-byte input:
    // Raw: 173 bytes + ceil(173/254)=1 overhead byte + 2 frame delimiters = 176 bytes
    // Buffer: 300 bytes (50% margin for protocol version flexibility)
    static constexpr size_t kEncBufSize = 300;

    // Returns number of bytes written to out_buf (including both 0x00 frame delimiters)
    static size_t encode(const uint8_t* input, size_t in_len,
                         uint8_t* out_buf, size_t out_buf_size);

    // Full pipeline: encode SignedEtPacket → COBS → CDC_Transmit_FS
    // Returns true if USB CDC accepted the transmit request.
    bool transmit(const SignedEtPacket& pkt);

private:
    alignas(4) static uint8_t enc_buf_[kEncBufSize];
};

// ── COBS encode implementation ─────────────────────────────────────────────
// Produces: [0x00][COBS encoded data][0x00]
// Contract: out_buf_size must be >= in_len + ceil(in_len/254) + 2
size_t CobsTxDriver::encode(const uint8_t* input, size_t in_len,
                             uint8_t* out_buf, size_t out_buf_size) {
    out_buf[0] = 0x00;              // start-of-frame delimiter
    size_t out_pos  = 2;            // out_buf[1] = first code byte (filled below)
    size_t code_pos = 1;            // position of current overhead byte
    uint8_t code    = 1;            // counts bytes until next zero (or block end)

    for (size_t i = 0; i < in_len; ++i) {
        if (input[i] == 0x00) {
            out_buf[code_pos] = code;   // finalize current block
            code_pos = out_pos++;       // next overhead byte
            code = 1;
        } else {
            out_buf[out_pos++] = input[i];
            ++code;
            if (code == 0xFF) {
                out_buf[code_pos] = code;   // block of 254 non-zero bytes
                code_pos = out_pos++;
                code = 1;
            }
        }
    }
    out_buf[code_pos] = code;       // finalize last block
    out_buf[out_pos++] = 0x00;     // end-of-frame delimiter
    return out_pos;                 // total bytes written
}
```

#### 6.4.3 USB-CDC Transmission

```cpp
bool CobsTxDriver::transmit(const SignedEtPacket& pkt) {
    // Serialize struct to raw bytes (already packed — direct cast is valid on LE ARM)
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt);

    // COBS encode into static buffer
    const size_t encoded_len = encode(raw, sizeof(SignedEtPacket),
                                      enc_buf_, kEncBufSize);

    // CDC_Transmit_FS is non-blocking — it copies data into the USB endpoint buffer
    // and returns USBD_OK immediately if the USB TX FIFO is available.
    // Returns USBD_BUSY if previous transmit not yet complete.
    const uint8_t result = CDC_Transmit_FS(enc_buf_, static_cast<uint16_t>(encoded_len));

    // If BUSY: data is dropped for this cycle. At 10 Hz, the next cycle retries.
    // This is acceptable — the Pi side handles missing sequence numbers gracefully.
    return (result == USBD_OK);
}
```

---

## 7. Main Loop Architecture

```cpp
// firmware/src/main_loop.cpp

extern "C" void vigia_main_loop() {
    // ── One-time initialization ────────────────────────────────────────────
    tim2_init_microsecond_counter();    // TIM2 free-running @ 1 MHz, 32-bit

    bno085.init(&hspi1);               // SPI1 DMA init + BNO085 boot + Set Feature cmds
    gps.init(&huart2);                 // USART2 DMA circular mode + IDLE interrupt
    atecc.init();                      // cryptoauthlib init + ATECC608A wake

    led_set(LED_STATUS, true);         // green: system ready

    uint32_t packet_sequence = 0;

    // ── Super-loop ────────────────────────────────────────────────────────
    while (true) {
        __WFI();   // Sleep until next interrupt (DMA TC, UART IDLE, USB OTG FS)

        // ── BNO085 frame ready (set by DMA2 Stream0 TC ISR) ───────────────
        if (g_bno085_frame_ready.load(std::memory_order_acquire)) {
            bno085.process();   // parse SHTP, update latest_report_
            // No further action — report consumed on next GPS trigger below
        }

        // ── GPS frame ready (set by USART2 IDLE ISR) → sign + transmit ───
        if (g_gps_frame_ready.load(std::memory_order_acquire)) {
            g_gps_frame_ready.store(false, std::memory_order_relaxed);

            gps.process();     // drain ring buffer, parse NAV-PVT

            Bno085Driver::Report  imu_report{};
            NeoM8nDriver::NavPvtReport gps_report{};

            const bool imu_ok = bno085.get_report(imu_report);
            const bool gps_ok = gps.get_report(gps_report);

            if (!imu_ok || !gps_ok) {
                // Missing sensor data — skip this cycle, do not transmit
                // LED blink pattern: fast blink = waiting for sensor lock
                continue;
            }

            // ── Build E_t hash input ──────────────────────────────────────
            EtHashInput et{};
            // device_id read from ATECC608A UserExtra zone at init and cached
            std::memcpy(et.device_id, atecc.device_id(), 16);
            et.timestamp_us  = tim2_get_microseconds();
            et.sequence      = ++packet_sequence;
            et.q_w           = imu_report.q_w;
            et.q_x           = imu_report.q_x;
            et.q_y           = imu_report.q_y;
            et.q_z           = imu_report.q_z;
            et.lin_accel_x   = imu_report.lin_accel_x;
            et.lin_accel_y   = imu_report.lin_accel_y;
            et.lin_accel_z   = imu_report.lin_accel_z;
            et.imu_cal_status = imu_report.calibration_status;
            std::memset(&et._pad0, 0, sizeof(et._pad0));   // zero explicit padding
            et.latitude      = gps_report.latitude;
            et.longitude     = gps_report.longitude;
            et.altitude_m    = gps_report.altitude_m;
            et.speed_ms      = gps_report.speed_ms;
            et.course_deg    = gps_report.course_deg;
            et.fix_type      = gps_report.fix_type;
            et.satellites    = gps_report.satellites;
            std::memset(&et._pad1, 0, sizeof(et._pad1));
            et.hdop          = gps_report.hdop;

            // ── SHA-256 + ECDSA sign (blocking, ~57 ms) ──────────────────
            uint8_t hash[32]{};
            uint8_t sig[64]{};
            if (!atecc.sign(et, hash, sig)) {
                led_set(LED_ERROR, true);    // red LED: signing failure
                continue;
            }
            led_set(LED_ERROR, false);

            // ── Assemble SIGNED_ET packet ─────────────────────────────────
            SignedEtPacket pkt{};
            pkt.version      = 0x01;
            pkt.type         = 0x03;
            pkt.sequence     = et.sequence;
            pkt.timestamp_us = et.timestamp_us;
            pkt.q_w = et.q_w; pkt.q_x = et.q_x;
            pkt.q_y = et.q_y; pkt.q_z = et.q_z;
            pkt.lin_accel_x  = et.lin_accel_x;
            pkt.lin_accel_y  = et.lin_accel_y;
            pkt.lin_accel_z  = et.lin_accel_z;
            pkt.imu_cal_status = et.imu_cal_status;
            pkt.latitude     = et.latitude;
            pkt.longitude    = et.longitude;
            pkt.altitude_m   = et.altitude_m;
            pkt.speed_ms     = et.speed_ms;
            pkt.course_deg   = et.course_deg;
            pkt.fix_type     = et.fix_type;
            pkt.satellites   = et.satellites;
            std::memset(&pkt._gps_pad, 0, sizeof(pkt._gps_pad));
            pkt.hdop         = et.hdop;
            std::memcpy(pkt.et_hash, hash, 32);
            std::memcpy(pkt.ecdsa_sig, sig, 64);

            // ── COBS encode + USB-CDC transmit (non-blocking) ────────────
            cobs_tx.transmit(pkt);

            // ── Toggle status LED (heartbeat: 1 Hz if GPS runs at 10 Hz) ─
            if (packet_sequence % 10 == 0) {
                led_toggle(LED_STATUS);
            }
        }
    }
}
```

---

## 8. `operator new` Deletion — Link-Time Enforcement

Place this in `firmware/src/no_heap.cpp`. The linker error fires at compile time if any TU attempts dynamic allocation.

```cpp
// firmware/src/no_heap.cpp
#include <cstdlib>

// Block all C++ dynamic allocation at link time.
// Any TU that calls new/delete/malloc/free will get a linker error
// pointing here, not a silent runtime heap corruption.

void* operator new  (std::size_t) noexcept { return nullptr; }
void* operator new[](std::size_t) noexcept { return nullptr; }
void  operator delete  (void*)    noexcept {}
void  operator delete[](void*)    noexcept {}
void  operator delete  (void*, std::size_t) noexcept {}
void  operator delete[](void*, std::size_t) noexcept {}

// Override malloc/free to catch C-style allocation from third-party code
extern "C" {
    void* malloc(std::size_t)      { return nullptr; }
    void* calloc(std::size_t, std::size_t) { return nullptr; }
    void* realloc(void*, std::size_t)      { return nullptr; }
    void  free(void*)              {}
}

// Exception: cryptoauthlib with ATCA_NO_HEAP defined bypasses these.
// Verify ATCA_NO_HEAP is set in atca_config.h before including cryptoauthlib.
```

---

## 9. Firmware Build Configuration

### 9.1 `CMakeLists.txt` (ARM bare-metal)

```cmake
cmake_minimum_required(VERSION 3.24)
project(vigia_stm32 LANGUAGES C CXX ASM)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
set(CMAKE_SIZE         arm-none-eabi-size)

# ── CPU flags ──────────────────────────────────────────────────────────────
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard        # FPU registers used — mandatory for float performance
)

# ── Compiler flags ─────────────────────────────────────────────────────────
set(COMMON_FLAGS
    ${MCU_FLAGS}
    -O2                     # Optimize for speed (not -O3 — code size matters on 512KB Flash)
    -ffunction-sections     # Dead code elimination per function
    -fdata-sections
    -fno-exceptions         # No C++ exceptions — saves ~20KB Flash
    -fno-rtti               # No runtime type info — saves ~5KB Flash
    -Wall -Wextra
    -Wno-unused-parameter
)

set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(${COMMON_FLAGS})

# ── Linker flags ───────────────────────────────────────────────────────────
add_link_options(
    ${MCU_FLAGS}
    -T ${CMAKE_SOURCE_DIR}/STM32F411CEUx_FLASH.ld
    -Wl,--gc-sections       # Remove dead code sections
    -Wl,-Map=${PROJECT_NAME}.map
    -specs=nano.specs       # newlib-nano (smaller printf, no float printf)
    -specs=nosys.specs      # No syscall stubs (no OS)
    -lc -lm                 # C std + math (newlib-nano)
)

# ── Sources ────────────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME}
    src/main.c               # CubeMX-generated C entry point — calls vigia_main_loop()
    src/main_loop.cpp
    src/bno085_driver.cpp
    src/neo_m8n_driver.cpp
    src/atecc608a_driver.cpp
    src/cobs_tx_driver.cpp
    src/no_heap.cpp
    src/stm32f4xx_it.cpp     # ISR handlers (EXTI0, DMA2_Stream0, USART2)
    # STM32 HAL (generated by CubeMX — do not modify):
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    # USB CDC middleware (ST USB device library):
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/App/usbd_desc.c
    # cryptoauthlib (Microchip — ATCA_NO_HEAP must be defined):
    third_party/cryptoauthlib/lib/atca_command.c
    third_party/cryptoauthlib/lib/atca_device.c
    third_party/cryptoauthlib/lib/atca_execution.c
    third_party/cryptoauthlib/lib/atca_iface.c
    third_party/cryptoauthlib/lib/basic/atca_basic.c
    third_party/cryptoauthlib/lib/basic/atca_basic_sha.c
    third_party/cryptoauthlib/lib/basic/atca_basic_sign.c
    third_party/cryptoauthlib/hal/atca_hal_stm32.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    src/
    Core/Inc/
    Drivers/STM32F4xx_HAL_Driver/Inc/
    Drivers/CMSIS/Device/ST/STM32F4xx/Include/
    Drivers/CMSIS/Include/
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc/
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/
    USB_DEVICE/App/ USB_DEVICE/Target/
    third_party/cryptoauthlib/lib/
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    STM32F411xE
    USE_HAL_DRIVER
    ATCA_NO_HEAP            # cryptoauthlib: disable internal malloc
    ATCA_HAL_I2C            # cryptoauthlib: use I2C HAL
    ATCA_ATECC608A_SUPPORT  # cryptoauthlib: enable ATECC608A features
)

# ── Post-build: generate .bin and print size ───────────────────────────────
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
        $<TARGET_FILE:${PROJECT_NAME}>
        ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating ${PROJECT_NAME}.bin"
)
```

### 9.2 Flash Size Budget

| Region | Size | Notes |
|---|---|---|
| STM32 HAL (C) | ~60 KB | CubeMX minimal config |
| USB CDC middleware | ~20 KB | ST USB device library |
| cryptoauthlib | ~40 KB | With `ATCA_NO_HEAP`, SHA + Sign only |
| Application C++ | ~30 KB | 4 driver classes + main loop |
| **Total Flash** | **~150 KB** | / 512 KB available = 29% utilized |

---

## 10. IRQ Priority Table

All NVIC priorities configured before peripherals are enabled.

```c
// firmware/src/main.c — IRQ priority configuration
HAL_NVIC_SetPriority(OTG_FS_IRQn,        1, 0);  // USB — highest: never delay USB stack
HAL_NVIC_SetPriority(EXTI0_IRQn,         2, 0);  // BNO085 INT
HAL_NVIC_SetPriority(DMA2_Stream0_IRQn,  3, 0);  // SPI1 RX DMA TC
HAL_NVIC_SetPriority(DMA2_Stream3_IRQn,  3, 1);  // SPI1 TX DMA TC (lower sub-priority)
HAL_NVIC_SetPriority(USART2_IRQn,        4, 0);  // GPS IDLE line
HAL_NVIC_SetPriority(TIM2_IRQn,          5, 0);  // µs tick overflow (lowest)

// SysTick for HAL_Delay (used only during init — not in super-loop)
HAL_NVIC_SetPriority(SysTick_IRQn,       6, 0);
```

> **Priority inversion risk:** The ATECC608A I2C signing uses `HAL_I2C_Master_Transmit()` (blocking). During the ~57 ms signing window, `__WFI()` is NOT called — the CPU polls the I2C busy flag. All interrupts (DMA TC, USART IDLE) still fire and set their atomic flags normally. The BNO085 DMA may complete once during the signing window; its frame will be processed on the next super-loop iteration. This is acceptable — the IMU data for the next GPS cycle will be fresher anyway.

---

## 11. File Structure

```
firmware/
├── CMakeLists.txt
├── STM32F411CEUx_FLASH.ld          # Linker script (CubeMX generated)
├── Core/
│   ├── Inc/
│   │   └── main.h                  # CubeMX generated
│   └── Src/
│       └── main.c                  # CubeMX entry — calls vigia_main_loop()
├── src/                            # Hand-written C++ firmware
│   ├── main_loop.cpp               # Super-loop: orchestration
│   ├── bno085_driver.hpp/.cpp
│   ├── neo_m8n_driver.hpp/.cpp
│   ├── atecc608a_driver.hpp/.cpp
│   ├── cobs_tx_driver.hpp/.cpp
│   ├── no_heap.cpp                 # operator new override — link-time heap guard
│   ├── isr_flags.hpp               # Atomic cross-ISR flags
│   └── stm32f4xx_it.cpp            # ISR implementations (replaces CubeMX .c)
├── Drivers/                        # CubeMX generated — do not hand-edit
│   ├── STM32F4xx_HAL_Driver/
│   └── CMSIS/
├── Middlewares/                    # ST USB device library
│   └── ST/STM32_USB_Device_Library/
├── USB_DEVICE/                     # CubeMX USB CDC configuration
│   ├── App/
│   └── Target/
└── third_party/
    └── cryptoauthlib/              # Microchip cryptoauthlib (pinned to v3.7.x)
        ├── lib/
        └── hal/
            └── atca_hal_stm32.c   # Hand-written STM32 HAL layer
```

---

## 12. Acceptance Criteria

| Test | Method | Pass Condition |
|---|---|---|
| **No dynamic allocation** | `arm-none-eabi-nm firmware.elf \| grep ' malloc\b'` | Zero results — `malloc` symbol not present in binary |
| **No RTOS symbols** | `arm-none-eabi-nm firmware.elf \| grep -E 'vTask\|osThread\|xQueue'` | Zero results |
| **Flash utilization** | `arm-none-eabi-size firmware.elf` | `.text` ≤ 300 KB (leaves 200 KB margin) |
| **SRAM utilization** | `arm-none-eabi-size firmware.elf` | `.bss` + `.data` ≤ 60 KB (leaves 68 KB for stacks/USB) |
| **BNO085 data rate** | Logic analyzer on PA4 (CS) — count falling edges per second | ≥ 95 CS pulses/sec (95 Hz IMU rate) |
| **GPS parse rate** | UART monitor — count valid NAV-PVT frames per second | ≥ 9 Hz |
| **ECDSA sign latency** | TIM2 timestamp delta: signing entry → `CDC_Transmit_FS` call | ≤ 75 ms |
| **COBS framing** | Pi-side `SensorBridgeNode` — count decode errors per 1000 packets | ≤ 1 error (0.1% loss budget) |
| **Anti-replay** | Replay same packet twice to Pi | `SensorBridgeNode` drops second packet, logs WARN |
| **ECDSA verification** | Pi-side mbedTLS verify over 1000 packets | 0 verification failures |
| **USB heartbeat** | `lsusb` on Pi shows `STMicroelectronics Virtual COM Port` | Device enumerated within 5s of power-on |

---

*Next document: `.claude/design/04_onnx_vision_engine_contracts.md` — ONNX Runtime + KleidiAI integration spec: `VisionNode` session options, INT8 quantization workflow for YOLOv26, FP32 lock for MiDaS, penultimate feature map extraction procedure, and the S_t → `SpatialLatent` pipeline.*
