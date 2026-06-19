---
title: "NEO-M8N GPS"
type: hardware
tags: [hardware, sensor]
source: firmware/src/neo_m8n_driver.h
related: ["[[pico-2]]", "[[neo-m8n-driver]]", "[[sensor-bridge-node]]", "[[fusion-node]]", "[[et-hash-input]]"]
updated: 2026-06-19
---

# NEO-M8N GPS Module (u-blox)

**Interface:** UART1 on Pico 2 @ 9600 baud  
**Pins:** TX=GP8 (config commands), RX=GP9 (UBX data)  
**Protocol:** UBX binary (NAV-PVT messages)  
**Data Rate:** 1 Hz fix (firmware target: 10 Hz via UBX config command)

## UBX NAV-PVT Message
Class 0x01, ID 0x07 — 100 bytes total (92 bytes payload).  
Parser extracts: latitude, longitude (deg×1e-7), gSpeed (mm/s→m/s), headMot, pDOP×0.01, fixType, numSV, gnssFixOK flag.

## Output Fields
| Field | Format | Notes |
|---|---|---|
| `latitude` | float64 degrees WGS-84 | |
| `longitude` | float64 degrees WGS-84 | |
| `altitude_m` | float32 m above ellipsoid | |
| `speed_ms` | float32 m/s ground speed | |
| `course_deg` | float32 degrees (0=N, CW) | |
| `fix_type` | uint8 | 0=none, 2=2D, 3=3D, 4=GNSS+DR |
| `satellites` | uint8 | |
| `hdop` | float32 | |
| `valid` | bool | gnssFixOK && hdop <= 2.5 |

## Communication via Pico 2
- UART1 RX → ring buffer (512 B) → DMA channel 2 circular
- UART RX timeout ISR → `g_gps_frame_ready` → super-loop `process()` → Fletcher-8 validated UBX parse
- Data via [[cobs-usb-cdc]] → [[sensor-bridge-node]] → `/vigia/gps` topic @ 10 Hz

## Use in Fusion
- [[fusion-node]] uses GPS speed for ISS denominator: `ISS = |a_z| / max(v_GPS, v_min=2.0)`
- Kalman filter update step when `valid_fix && hdop <= 2.5`
- Dead-reckoning via IMU when GPS invalid

## Links
- Connected to: [[pico-2]] via UART1
- Driver: [[neo-m8n-driver]]
- Data path: [[sensor-bridge-node]] → [[fusion-node]]
- ABI: [[et-hash-input]] (GPS fields in signed struct)
