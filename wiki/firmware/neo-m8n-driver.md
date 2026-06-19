---
title: "NeoM8nDriver"
type: firmware
tags: [firmware, sensor]
source: firmware/src/neo_m8n_driver.c
related: ["[[pico-2]]", "[[neo-m8n]]", "[[sensor-bridge-node]]", "[[et-hash-input]]", "[[cobs-tx-driver]]", "[[bno085-driver]]"]
updated: 2026-06-19
---

# NeoM8nDriver

**Files:** `firmware/src/neo_m8n_driver.c` / `firmware/src/neo_m8n_driver.h`  
**Status:** Built and flashed

UART1 ring-buffer UBX NAV-PVT parser for [[neo-m8n]] at 1-10 Hz GPS fix output.

## Interface
- **Bus:** UART1, 9600 baud
- **Pins:** TX=GP8 (reserved for config), RX=GP9
- DMA channel 2 (circular ring buffer, UART1 RX)

## `NeoM8nDriver::NavPvtReport` struct
```cpp
struct NavPvtReport {
    double  latitude, longitude;   // degrees WGS-84
    float   altitude_m, speed_ms, course_deg, hdop;
    uint8_t fix_type, satellites;
    bool    valid;                 // gnssFixOK bit from flags@21
};
```

## UBX NAV-PVT Frame (100 bytes total)
```
[0xB5][0x62]         sync chars
[0x01][0x07]         Class=NAV, ID=PVT
[Length LSB/MSB]     92 bytes payload
[92 bytes payload]
[CK_A][CK_B]         Fletcher-8 checksum
```

## Ring Buffer Drain Algorithm
UART1 RX timeout IRQ → sets `g_gps_frame_ready`, `g_gps_ring_write_pos`  
Super-loop `process()`:
1. Copy available bytes from 512-byte ring (handle wrap-around)
2. Scan for sync `[0xB5 0x62 0x01 0x07]`
3. Verify length field == 92
4. Validate Fletcher-8 checksum
5. `parse_nav_pvt(payload)` — extract fields from byte offsets

## NAV-PVT Field Offsets (UBX Protocol §32.17.14.1)
| Field | Offset | Format |
|---|---|---|
| `fixType` | 20 | uint8 |
| `flags` (gnssFixOK = bit 0) | 21 | uint8 |
| `numSV` (satellites) | 23 | uint8 |
| `lon` (deg × 1e-7) | 24 | int32 LE |
| `lat` (deg × 1e-7) | 28 | int32 LE |
| `gSpeed` (mm/s) | 60 | int32 LE |
| `headMot` (deg × 1e-5) | 64 | int32 LE |
| `pDOP` (× 0.01) | 76 | uint16 LE |

## Static Buffers
- `ring_buf_[512]` — UART1 RX ring
- `ubx_frame_buf_[128]` — UBX parse buffer

## Links
- Hardware: [[neo-m8n]] (UART1 GPS)
- MCU: [[pico-2]] (RP2350)
- Data used by: [[sensor-bridge-node]] (Pi-side), [[et-hash-input]] (ABI)
- Next: [[cobs-tx-driver]]
- Related: [[bno085-driver]]
