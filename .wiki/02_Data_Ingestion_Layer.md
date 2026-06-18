# Data Ingestion Layer

Defines how streaming packet feeds arrive from the Pico MCU, are decoded, validated,
and deposited into the `SensorState` ring buffer for consumption by the Coordinator.

## Pipeline Overview

```
Pico USB-UART
    │
    ▼
SensorBridge::readLoop()         ← dedicated thread, blocked on select(200 ms)
    │
    ├── WireProto::Text  ──▶ pending[] accumulator (cap: 4 096 B)
    │       │ newline found
    │       ▼
    │   processLine()
    │       ├── parseImuLine()   → ImuSample    → SensorState::updateImu()
    │       ├── parseGpsLine()   → GpsFix       → SensorState::updateGps()
    │       └── parsePingLine()  → PingReport   → SensorState::updateHealth()
    │
    └── WireProto::Cobs  ──▶ cobs_acc[] accumulator (cap: 512 B)
            │ 0x00 delimiter received
            ▼
        cobsDecode() → raw[173]
            │ dec_len == kSignedEtPacketSize (173)?
            ▼
        parseSignedEtPacket()   → SignedEtSample
            │
            ▼
        EcdsaVerifier::verify()  (secp256r1, optional)
            │
            ▼
        SensorState::updateSignedEt()
```

## Text Protocol Format (WireProto::Text)

Lines begin with `VIGIA_` prefix. Unrecognised `VIGIA_*` lines increment `parse_errors`.

```
VIGIA_IMU seq=<u32> timestamp_us=<u64> qw=<f> qx=<f> qy=<f> qz=<f>
          ax=<f> ay=<f> az=<f> cal=<u> valid=<u> qnorm=<f>

VIGIA_GPS seq=<u32> timestamp_us=<u64> lat=<f64> lon=<f64> speed_ms=<f>
          fix_type=<u> satellites=<u> hdop=<f> valid=<u> src=<str31>

VIGIA_PING seq=<u32> uptime_ms=<u64> boot_ms=<u64> [...]
```

`parseGpsLine()` accepts a legacy variant without `timestamp_us` / `src` fields
(matched by the 8-field fallback `sscanf` path).

## COBS Binary Protocol (WireProto::Cobs)

- Frame structure: `[0x00][<COBS-encoded 173 bytes>][0x00]`
- `cobsDecode()` returns 0 on any bound violation; caller discards and increments
  `parse_errors`.
- Payload: `SignedEtPacketView` (173 bytes, `#pragma pack(push,1)`)
  - magic `0xE7`, version `0x02`
  - Fields: timestamp_us, sequence, quaternion, accelerometer, lat/lon, ECDSA hash + sig
- Encode capacity: `out_cap ≥ src_len + ⌈src_len / 254⌉ + 3`
  (corrected from incorrect `src_len + 4` formula, June 2026)

## Sequence Gap Tracking

All three streams (IMU, GPS, SignedEt) maintain `last_*_seq` and `*_seq_gaps`
counters in `SensorHealth`. A gap is recorded only when the new sequence
**advances forward past the expected next value** — wrap-around (e.g.,
`0xFFFFFFFF → 0`) is not counted.

> **Bug fixed June 2026**: `handleSignedEt()` previously used `<= last_seq` which
> fired on every valid wrap and on duplicate delivery, polluting `parse_errors`.
> Now mirrors the IMU/GPS gap-tracking pattern.

## Disconnect Recovery

When `select()` or `read()` returns a hard error, or `read()` returns 0 (EOF),
`closeSerial()` is called and the loop waits `reconnect_delay_ms` before
re-attempting `openSerial()`. Protocol state and both accumulators are cleared
on each reconnect. This survives a Pico power cycle, cable reconnect, or USB
re-enumeration with no manual restart required.

Backlinks: [[01_Hardware_Constraints]] | [[03_Telemetry_Signing]] | [[00_Index]]
