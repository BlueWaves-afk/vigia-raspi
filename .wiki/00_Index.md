# Systems Knowledge Graph Index

## System Architecture Nodes

| Note | Covers |
|------|--------|
| [[01_Hardware_Constraints]] | Raspberry Pi 5 / Cortex-A76, sensor hardware, thermal limits |
| [[02_Data_Ingestion_Layer]] | Serial ingestion, COBS framing, text-protocol parsing |
| [[03_Telemetry_Signing]] | ECDSA signature verification, HMAC event signing, key loading |
| [[04_Coordinator_And_Threads]] | Thread topology, CPU affinity, frame pipeline, thermal throttle |
| [[05_Event_Pipeline]] | HazardObservation lifecycle, EventPromoter, EventStore, HTTP sync |
| [[06_Fusion_Engine]] | Multi-modal score fusion (YOLO + MiDaS + IMU-ISS + GPS) |
| [[07_Firmware_Bridge]] | Pico firmware, sensor drivers, COBS transmit, ATECC608A signing |

## Inter-Module Data Flow

```
Camera ──▶ Coordinator.captureLoop (Core 0)
                │
                ▼
         Coordinator.processLoop (Core 1)
                │ YOLO inference (OpenVINO)
                │ querySensors → SensorBridge → SensorState
                │
                ├──▶ midasQueue (SafeQueue<MidasWork>)
                │           │
                │           ▼  Coordinator.midasLoop (Core 2)
                │         MiDaS depth inference → FusionEngine → EventStore
                │
                ▼
         SensorBridge.readLoop (dedicated thread)
           ├── WireProto::Text  → parseImuLine / parseGpsLine / parsePingLine
           └── WireProto::Cobs  → cobsDecode → parseSignedEtPacket → EcdsaVerifier

EventStore.syncLoop ──▶ EventPromoter ring buffer ──▶ SyncClient (libcurl / stdout)
```

## Build System
- Root: `CMakeLists.txt` — Cortex-A76 `-O3 -ftree-vectorize`, optional libcurl + OpenSSL
- Standalone test gate: `make test` / `make validate-buffers`
- Firmware: `firmware/CMakeLists.txt` — RP2040 Pico SDK + cryptoauthlib

#tags: #edge-compute #embedded-systems #vigia-stack
