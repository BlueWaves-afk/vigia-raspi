---
title: "Bno085Driver"
type: firmware
tags: [firmware, sensor]
source: firmware/src/bno085_driver.c
related: ["[[pico-2]]", "[[bno085]]", "[[sensor-bridge-node]]", "[[et-hash-input]]", "[[cobs-tx-driver]]", "[[neo-m8n-driver]]"]
updated: 2026-06-19
---

# Bno085Driver

**Files:** `firmware/src/bno085_driver.c` / `firmware/src/bno085_driver.h`  
**Status:** Built and flashed (text protocol output over USB CDC)

SPI0 DMA-driven SHTP driver for [[bno085]] at 100 Hz quaternion + linear acceleration output. Uses interrupt-driven DMA — no parsing in ISR.

## Interface
- **Bus:** SPI0, CPOL=1 CPHA=1 (Mode 3), 3.0 MHz (BNO085 max — do not exceed)
- **Pins:** SCK=GP18, MOSI=GP19, MISO=GP16, CS=GP17, INT=GP20, RST=GP21, WAKE=GP22
- Defined in `firmware/src/vigia_pins.h`

## `Bno085Driver::Report` struct
```cpp
struct Report {
    float q_w, q_x, q_y, q_z;           // Unit quaternion (world←body)
    float lin_accel_x, lin_accel_y, lin_accel_z;  // Body-frame m/s²
    uint8_t calibration_status;          // 0=uncal, 3=fully cal
    bool valid;
};
```

## Fixed-Point Conversion (Q-point)
- Quaternion: Q14 → float: `int16 × (1.0f / 16384.0f)`
- Linear accel: Q8 → float: `int16 × (1.0f / 256.0f)`, units m/s²

## ISR → DMA → Super-Loop Flow
1. BNO085_INT (GP20) falling edge IRQ → assert CS (GP17 LOW) → start SPI0 DMA (512 bytes)
2. DMA RX complete IRQ → de-assert CS → `g_bno085_frame_ready.store(true)`
3. Super-loop: `if (g_bno085_frame_ready)` → `bno085.process()` → `parse_shtp_frame()` → parse rotation vector (0x05) + linear accel (0x04) report IDs

## Initialization Sequence
1. Assert RST (GP21) LOW 10 ms → release HIGH
2. Wait BNO085_INT falling edge
3. Read SHTP channel 0 boot advertisement (discard)
4. Send Set Feature: Rotation Vector (0x05) @ 10000 µs (100 Hz)
5. Send Set Feature: Linear Acceleration (0x04) @ 10000 µs (100 Hz)
6. Enable GPIO falling-edge IRQ on GP20

## Static Buffers
- `rx_buf_[512]` / `tx_buf_[512]` (static, 4-byte aligned)
- DMA channel 0 (RX) + channel 1 (TX)

## Links
- Hardware: [[bno085]] (I2C/SPI physical device)
- MCU: [[pico-2]] (RP2350)
- Data used by: [[sensor-bridge-node]] (Pi-side), [[et-hash-input]] (ABI)
- Next: [[cobs-tx-driver]] (encodes output into COBS packet)
- Related: [[neo-m8n-driver]]
