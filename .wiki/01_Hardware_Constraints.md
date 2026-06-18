# Hardware Constraints

## Compute Platform — Raspberry Pi 5

| Parameter | Value |
|-----------|-------|
| SoC | BCM2712 |
| CPU | 4× Cortex-A76 @ 2.4 GHz |
| ISA | ARMv8.2-A / AArch64 |
| SIMD | NEON (FP16 dot-product via FEAT_DotProd) |
| Compiler flags | `-mcpu=cortex-a76 -O3 -ftree-vectorize` |
| OS | Debian Trixie (Linux aarch64) |
| Thermal warn | 75 °C — MiDaS stride increases to 3 |
| Thermal critical | 85 °C — MiDaS stride increases to 5 |
| Temp sensor | `/sys/class/thermal/thermal_zone0/temp` (read every 1 s) |

## Thread → Core Affinity

| Thread | Core | Rationale |
|--------|------|-----------|
| `captureLoop` | 0 | Dedicated camera decode — keeps VideoCapture L1 warm |
| `processLoop` | 1 | YOLO inference (ACL uses all cores internally; do not pin further) |
| `midasLoop` | 2 | MiDaS depth inference, async from YOLO |
| `SensorBridge::readLoop` | (unbound) | Low-rate serial I/O, blocked in `select()` most of the time |
| `EventStore::syncLoop` | (unbound) | Network-blocked HTTP POST |

## Sensor Hardware

| Device | Interface | MCU |
|--------|-----------|-----|
| BNO085 IMU | I²C | Raspberry Pi Pico (RP2040) |
| NEO-M8N GPS | UART → Pico | Raspberry Pi Pico (RP2040) |
| ATECC608A | I²C | Raspberry Pi Pico (RP2040) |
| Camera | CSI / USB | Direct to Pi 5 |

## Serial Link (Pi 5 ↔ Pico)

- Default device: `/dev/ttyACM0`
- Default baud: 115 200
- Wire protocol auto-detected per first received byte:
  - `0x56` ('V') → `WireProto::Text` (ASCII NMEA-like lines)
  - `0x00` → `WireProto::Cobs` (binary signed-et packets)
- **Disconnect recovery**: `SensorBridge::readLoop` calls `closeSerial()` on
  EOF or I/O error, then re-tries `openSerial()` after `reconnect_delay_ms`
  (default 2 000 ms) while `running_` is true.

## Buffer Limits (hardening additions — June 2026)

| Guard | Default cap | Action on overflow |
|-------|-------------|-------------------|
| `cobs_acc_` COBS frame accumulator | 512 bytes | Discard + `recordParseError()` |
| `pending` text line accumulator | 4 096 bytes | Flush + `recordParseError()` |

## External Dependencies

| Library | Purpose | Optional? |
|---------|---------|-----------|
| OpenCV 4.x | Frame capture, mat ops | No |
| OpenVINO 2025 | YOLO + MiDaS inference | No |
| OpenSSL | ECDSA verify, HMAC-SHA256, SHA-256 | Yes (`VIGIA_HAVE_OPENSSL`) |
| libcurl | HTTPS event POST | Yes (`VIGIA_HAVE_CURL`) |
| TBB | OpenVINO threading backend | Via OpenVINO |

Backlink: [[00_Index]]
