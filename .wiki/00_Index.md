# Systems Knowledge Graph Index

> [!NOTE] Architecture eras
> Notes **01–07** describe the edge device (Pi/Pico). The original demo used a
> monolithic `coordinator.cpp` (notes 04–05); the production runtime is now the
> **ROS 2 node graph** ([[08_ROS2_Edge_Nodes]]). The backend migrated to **AWS
> serverless** at M12 ([[09_Cloud_Pipeline]]) — the FastAPI/Mosquitto/PostgreSQL
> stack in [[05_Event_Pipeline]] is historical.

## Edge (Pi / Pico) Nodes

| Note | Covers |
|------|--------|
| [[01_Hardware_Constraints]] | Raspberry Pi 5 / Cortex-A76, sensor hardware, thermal limits |
| [[02_Data_Ingestion_Layer]] | Serial ingestion, COBS framing, text-protocol parsing |
| [[03_Telemetry_Signing]] | ECDSA signature verification, HMAC event signing, key loading |
| [[04_Coordinator_And_Threads]] | (Historical) monolith thread topology, CPU affinity, frame pipeline |
| [[05_Event_Pipeline]] | (Historical) HazardObservation lifecycle, EventPromoter, HTTP sync |
| [[06_Fusion_Engine]] | Multi-modal score fusion (YOLO + MiDaS + IMU-ISS + GPS) |
| [[07_Firmware_Bridge]] | Pico firmware, sensor drivers, COBS transmit, ATECC608A signing |
| [[08_ROS2_Edge_Nodes]] | **Current** ROS 2 node graph, SCHED_FIFO priorities, per-node config |

## Cloud (AWS) Nodes

| Note | Covers |
|------|--------|
| [[09_Cloud_Pipeline]] | AWS serverless backend (M12): IoT Core, Lambdas, DynamoDB, rewards |
| [[10_Cloud_Security_Model]] | Attestation chain, anti-replay, reward integrity, session-5 hardening |

## Cloud Data Flow (M12)

```
Pi hazard_uplink ──MQTT mTLS──▶ IoT Core ──Rule──▶ AttestationFn ─┐
Phone ──HTTPS Ed25519──▶ ValidatorFn ────────────────────────────┴──▶ HazardsTable
                                                       (stream INSERT) │
                                                                       ▼
                                            OrchestratorFn (2% VLM / 98% ONNX) ──▶ RewardsLedger + Solana
```

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

See also: [[09_Cloud_Pipeline]] for the AWS backend build/deploy (`vigia-amazon` CDK).

#tags: #edge-compute #embedded-systems #aws #depin #vigia-stack
