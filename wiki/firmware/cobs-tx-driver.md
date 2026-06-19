---
title: "CobsTxDriver"
type: firmware
tags: [firmware, transport]
source: firmware/src/cobs_tx_driver.c
related: ["[[pico-2]]", "[[atecc608a-driver]]", "[[bno085-driver]]", "[[neo-m8n-driver]]", "[[sensor-bridge-node]]", "[[cobs-usb-cdc]]", "[[et-hash-input]]"]
updated: 2026-06-19
---

# CobsTxDriver

**Files:** `firmware/src/cobs_tx_driver.c` / `firmware/src/cobs_tx_driver.h`  
**Status:** Written

No-heap COBS encoder + USB-CDC TinyUSB transmission for 173-byte `SignedEtPacket`.

## `SignedEtPacket` wire struct (173 bytes, `__attribute__((packed))`)
| Field | Size | Value |
|---|---|---|
| `magic` | uint8 | 0xE7 — Vigia DePIN frame marker |
| `version` | uint8 | 0x02 — Phase 2 protocol |
| `timestamp_us` | uint64 | Pico 2 µs timer |
| `sequence` | uint32 | Monotonic frame counter |
| `q_w/x/y/z` | float32 × 4 | BNO085 quaternion |
| `ax/ay/az` | float32 × 3 | linear accel |
| `cal_status` | uint8 + 3 pad | BNO085 cal |
| `latitude/longitude` | float64 × 2 | NEO-M8N |
| `speed_ms/fix_type/satellites/_gps_pad/hdop` | various | GPS |
| `et_hash[32]` | uint8[32] | SHA-256(EtHashInput) |
| `ecdsa_sig[64]` | uint8[64] | secp256r1 raw R||S |

`static_assert(sizeof(SignedEtPacket) == 173)`

## COBS Encoder (`encode_cobs`)
```c
// Produces: [0x00][COBS encoded data][0x00]
// No malloc. out must be >= src_len + ceil(src_len/254) + 2
size_t encode_cobs(const uint8_t* src, size_t src_len, uint8_t* out, size_t out_cap);
```
Static internal working buffer, not reentrant. Encoded frame up to ~175 bytes for 173-byte payload.

## USB-CDC Transmission (TinyUSB)
```c
bool transmit(const SignedEtPacket& pkt):
    encoded_len = encode_cobs(pkt, 173, enc_buf_, kEncBufSize)
    if (!tud_cdc_connected()) return false
    tud_cdc_write(enc_buf_, encoded_len)
    tud_cdc_write_flush()
```
Non-blocking: drops packet if TX FIFO full (acceptable at 1 Hz GPS-triggered rate).

## Static Memory
- `enc_buf_[300]` — COBS output buffer (static)
- DMA channel 0/1 used by SPI0 (BNO085), not UART TX

## Links
- Encodes output of: [[atecc608a-driver]], [[bno085-driver]], [[neo-m8n-driver]]
- Transport to Pi: [[cobs-usb-cdc]] via USB-CDC TinyUSB
- Decoded by: [[sensor-bridge-node]] (Pi-side COBS decoder)
- Struct ABI: [[et-hash-input]]
- MCU: [[pico-2]]
