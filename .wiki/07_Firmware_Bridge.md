# Firmware Bridge (Raspberry Pi Pico / RP2040)

Source tree: `firmware/` | `src/*.c` (C sensor drivers shared with Pi build)

## Sensor Drivers

| Driver | Hardware | Interface |
|--------|----------|-----------|
| `bno085_driver.c/.h` | BNO085 9-axis IMU | I²C (RP2040 hardware I²C) |
| `neo_m8n_driver.c/.h` | u-blox NEO-M8N GPS | UART (RP2040 UART0) |
| `atecc608a_driver.c/.h` | ATECC608A HSE | I²C (shared bus with BNO085) |
| `atca_hal_pico_i2c.c` | CryptoAuthLib HAL | I²C adapter for cryptoauthlib |
| `cobs_tx_driver.c/.h` | COBS framer / TX | USB-CDC or UART to Pi 5 |

## Data Flow (Pico → Pi)

```
BNO085 → bno085_driver → quaternion + accel (100 Hz)
                │
NEO-M8N → neo_m8n_driver → GpsFix (1–10 Hz)
                │
                ▼
        EtHashInput build   (IMU + GPS snapshot)
                │
        SHA-256 (software or ATECC608A hardware)
                │
        atcab_sign()  →  ECDSA R∥S (64 bytes)
                │
        SignedEtPacket assembly (173 bytes packed)
                │
        cobs_tx_driver → cobsEncode → USB-CDC write
```

## SignedEtPacket Wire Format (173 bytes, `#pragma pack(push,1)`)

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | magic `0xE7` |
| 1 | 1 | version `0x02` |
| 2 | 8 | `timestamp_us` (uint64, little-endian) |
| 10 | 4 | `sequence` (uint32) |
| 14 | 4×4 | `qw qx qy qz` (float32) |
| 30 | 3×4 | `ax ay az` (float32 m/s²) |
| 42 | 1 | `cal_status` |
| 43 | 3 | `imu_pad` (alignment) |
| 46 | 8 | `latitude` (double) |
| 54 | 8 | `longitude` (double) |
| 62 | 4 | `speed_ms` (float) |
| 66 | 1 | `fix_type` |
| 67 | 1 | `satellites` |
| 68 | 1 | `gps_pad` |
| 69 | 32 | `et_hash` (SHA-256) |
| 101 | 64 | `ecdsa_sig` (R∥S secp256r1) |
| 165 | 8 | `wire_pad` |
| **173** | | **total** |

`static_assert(sizeof(SignedEtPacketView) == 173)` enforces layout contract
between firmware and Pi host.

## COBS Framing

`cobs_tx_driver` calls `cobsEncode()` (shared `src/cobs.cpp`) to wrap the
173-byte payload. Frame on the wire: `[0x00][encoded bytes][0x00]`.

The Pi's `SensorBridge` auto-detects COBS mode from the first `0x00` byte and
uses a `cobs_acc_` accumulator (cap 512 B) to collect until the closing `0x00`.

## ATECC608A Key Provisioning

- The device private key is provisioned during factory setup and never leaves
  the HSE (ATECC608A enforces this in hardware).
- The corresponding public key (64-byte X∥Y) must be placed on the Pi in
  a hex file and configured as `SensorBridge::Config::pubkey_file`.
- Stub mode (all-zero sig) is available when `allow_stub_sig = true` — for
  development only; never deploy in production.

## Text Protocol (Alternative Mode)

When the Pico emits ASCII-only output (e.g., during early firmware boot or
when `cobs_tx_driver` is disabled), `SensorBridge` detects `WireProto::Text`
from the leading `V` byte and routes to the text line parser.
See [[02_Data_Ingestion_Layer]] for format details.

Backlinks: [[02_Data_Ingestion_Layer]] | [[03_Telemetry_Signing]] | [[01_Hardware_Constraints]] | [[00_Index]]
