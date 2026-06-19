---
title: "COBS USB-CDC (Pico→Pi)"
type: transport
tags: [transport]
source: firmware/src/cobs_tx_driver.c
related: ["[[pico-2]]", "[[raspberry-pi-5]]", "[[cobs-tx-driver]]", "[[sensor-bridge-node]]", "[[bno085-driver]]", "[[neo-m8n-driver]]", "[[atecc608a-driver]]", "[[flow-sensor-bridge]]"]
updated: 2026-06-19
---

# COBS USB-CDC Transport — Pico 2 → Pi 5

**Physical:** USB cable, Pico 2 native USB device controller (TinyUSB)  
**Pi side:** `/dev/ttyACM0` CDC ACM device  
**Baud rate:** 921600 bps (configured at Pi end; USB CDC is packet-based, baud is logical)  
**Protocol:** COBS (Consistent Overhead Byte Stuffing) binary framing

## Wire Format
```
0x00 [COBS-encoded SignedEtPacket (≤175 bytes)] 0x00
```
Framing: leading 0x00 delimiter + COBS-encoded payload + trailing 0x00 delimiter.  
COBS guarantees no 0x00 bytes inside the payload — reliable frame boundary detection.

## Packet: `SignedEtPacket` (173 bytes packed)
```
magic=0xE7, version=0x02
timestamp_us (uint64), sequence (uint32)
q_w/x/y/z (float32×4), ax/ay/az (float32×3), cal_status (uint8+3pad)
latitude (float64), longitude (float64), speed_ms (float32), fix_type, satellites, _pad[1], hdop
et_hash[32] = SHA-256(EtHashInput)
ecdsa_sig[64] = secp256r1 raw R||S
```
`static_assert(sizeof(SignedEtPacket) == 173)`

## Legacy Text Protocol (bringup/Phase 1)
```
VIGIA_IMU seq=N timestamp_us=T qw=W qx=X ... \n
VIGIA_GPS seq=N timestamp_us=T lat=L lon=G ... \n
VIGIA_PING uptime_ms=U firmware=V \n
```
Pi auto-detects via first byte: `'V'` (0x56) = text, other = COBS binary.

## Pi-Side Decoder (`sensor_bridge_node.cpp`)
`decode_cobs()` — pure state machine, no dynamic allocation, pre-allocated 512-byte buffer.  
`select()` + 256-byte chunk reads — 256× fewer syscalls vs byte-by-byte.

## Links
- Encoder: [[cobs-tx-driver]] (Pico 2)
- Decoder: [[sensor-bridge-node]] (Pi 5)
- Pico 2 source data: [[bno085-driver]], [[neo-m8n-driver]], [[atecc608a-driver]]
- Physical: [[pico-2]] → [[raspberry-pi-5]]
- Flow: [[flow-sensor-bridge]]
