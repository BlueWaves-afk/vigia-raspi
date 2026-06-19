---
title: "BleGattConstants"
type: cpp-class
tags: [cpp-class, transport, security]
source: vigia_ws/src/vigia_edge_node/include/vigia_edge_node/ble_gatt_constants.hpp
related: ["[[ble-gatt-node]]", "[[ble-frame-codec]]", "[[ble-gatt]]"]
updated: 2026-06-19
---

# BleGattConstants

**File:** `vigia_ws/src/vigia_edge_node/include/vigia_edge_node/ble_gatt_constants.hpp`

Namespace `vigia::ble`. GATT profile UUIDs and protocol bytes. Must stay byte-identical with Android app `GattConstants.kt`.

## Service & Characteristic UUIDs
| Name | UUID | Properties |
|---|---|---|
| Service | `5e355f98-eabf-4ae0-8417-919e926d411e` | — |
| `kHandshakeUuid` | `eb4b161b-3be6-4719-aa0f-8ef40bd44a36` | Read+Write+Notify |
| `kTelemetryUuid` | `4d231514-5514-4847-bb6d-64e3aa7a3ffb` | Notify (telemetry stream) |
| `kControlUuid` | `0bb821dd-6b24-4185-ad69-662510769d19` | Write (phone→Pi) |
| `kAttestUuid` | `580c5fb6-5283-4194-84c8-5d6aec75b88a` | Notify (anti-spoof, Phase 2) |
| `kCccdUuid` | `00002902-0000-1000-8000-00805f9b34fb` | CCCD standard |

## Handshake Protocol Bytes (`proto::`)
| Byte | Direction | Meaning |
|---|---|---|
| `0x01 kHello` | phone→Pi | initiate handshake |
| `0x02 kChallenge` | Pi→phone | nonce + Pi P-256 pubkey + sig |
| `0x03 kResponse` | phone→Pi | nonce + phone P-256 pubkey + sig |
| `0x04 kBound` | Pi→phone | CONFIRM success |
| `0xFF kErr` | Pi→phone | failure |

## Control Opcodes (`control::`)
`0x10` = Request256D, `0x11` = Request512D, `0x12` = PauseStream, `0x13` = ResumeStream, `0x20` = Rekey

## Links
- Used by: [[ble-gatt-node]]
- Codec: [[ble-frame-codec]]
- Transport: [[ble-gatt]]
