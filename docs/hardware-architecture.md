# Hardware Architecture

Team reference for the edge device hardware topology, component roles, data flows, security boundaries, and phased rollout.

**Target:** Raspberry Pi 5 edge computer + STM32 Black Pill sensor hub + BNO085 IMU + NEO-M8N GPS + ATECC608A secure element (MCU side only, Phase 1).

---

## System Block Diagram

```mermaid
flowchart TB
  subgraph vehicle [Vehicle Enclosure]
    subgraph power [Power Subsystem]
      input[12V Battery Input]
      protection[Fuse TVS Reverse Polarity]
      buck5v[5V Buck 5-6A]
      buck3v3[3.3V Low-Noise Sensor Rail]
      input --> protection --> buck5v
      protection --> buck3v3
    end

    subgraph compute [Compute Domain]
      camera[Pi Camera Module 3]
      pi5[Raspberry Pi 5 8GB]
      nvme[NVMe SSD via PCIe]
      lte[Optional LTE or WiFi]
      camera --> pi5
      nvme --> pi5
      lte --> pi5
      buck5v --> pi5
    end

    subgraph sensorHub [Sensor Domain]
      imu[BNO085 IMU]
      gps[NEO-M8N GPS]
      seMcu[ATECC608A Secure Element]
      stm32[STM32F411 Black Pill]
      antenna[Active GPS Antenna]
      buck3v3 --> stm32
      buck3v3 --> imu
      buck3v3 --> gps
      buck3v3 --> seMcu
      imu -->|I2C 400kHz| stm32
      gps -->|UART| stm32
      seMcu -->|I2C| stm32
      antenna --> gps
    end

    stm32 -->|USB CDC Serial| pi5
    stm32 -->|GPIO Heartbeat| pi5
    pi5 -->|GPIO Alive| stm32
  end

  subgraph cloud [Cloud Backend]
    ingest[Telemetry Ingest API]
    registry[Device Registry PKI]
    map[Road Health Map]
    ingest --> map
    registry --> ingest
  end

  pi5 -->|TLS 1.3 mTLS Software| ingest
```

---

## Compute Split

| Concern | Raspberry Pi 5 | STM32 Black Pill |
|---|---|---|
| Primary job | Vision AI, fusion, storage, networking | Real-time sensor sampling |
| Timing | Non-deterministic (Linux scheduler) | Deterministic (bare-metal / RTOS) |
| IMU rate | Cannot guarantee 100 Hz under inference load | Guaranteed 100 Hz BNO085 reads |
| GPS | Competes with other USB devices if attached directly | UART to STM32, shared timestamp domain |
| Accident detection (future) | Post-process clips, upload events | Detect impact spikes in microseconds |
| Watchdog | Can stall during heavy inference | Independent heartbeat to Pi |

**Power rule:** The Pi is a load, not the system power supply. Protected input feeds separate 5 V (Pi) and 3.3 V (sensor) rails.

---

## Component Reference

### Raspberry Pi 5

| Attribute | Detail |
|---|---|
| Role | Camera capture, YOLO + MiDaS inference, RRI fusion, local logging, cloud uplink |
| Software | C++17, OpenVINO 2025, OpenCV 4.x, KleidiAI INT8 |
| Storage | NVMe SSD via PCIe (preferred over SD-only for field logging) |
| Cooling | Active cooler required under sustained inference |
| Camera | Pi Camera Module 3 (CSI preferred) or secured USB camera |
| Telemetry hook | `Coordinator::publishResult()` in `src/coordinator.cpp` |

### STM32 Black Pill (STM32F411)

| Attribute | Detail |
|---|---|
| Role | BNO085 + GPS sampling, timestamping, packet framing, watchdog |
| Link to Pi | USB CDC — appears as `/dev/ttyACM0` |
| Debug | SWD header exposed for firmware updates |
| Firmware | Separate repo |

### BNO085 IMU

| Attribute | Detail |
|---|---|
| Role | Linear acceleration, angular velocity, orientation, gravity vector, vibration/impact evidence |
| Bus | I2C to STM32 at 400 kHz (not Pi) |
| Sample rate | ~100 Hz |
| Mounting | Rigid mount on vehicle frame; mark axis orientation permanently |
| Uses | Motion gating, geo-tag alignment, future crash spike detection |

### NEO-M8N GPS

| Attribute | Detail |
|---|---|
| Role | Lat/lon, UTC time, speed, course, fix quality, HDOP, satellite count |
| Bus | UART to STM32 |
| Sample rate | 1–10 Hz (5 Hz typical for hazard geo-tagging) |
| Antenna | Active antenna, clear sky view, away from Pi/LTE/DC-DC EMI |

### ATECC608A Secure Element (STM32 only — Phase 1)

| Attribute | Detail |
|---|---|
| Part | Microchip ATECC608A |
| Location | I2C on STM32 sensor bus (default address `0x60`) |
| Role | AEAD session key storage, MCU→Pi packet authentication, sensor hub identity |
| Library | Microchip CryptoAuthLib |
| Cost | ~$1–3 breakout at prototype scale |

**Phase 1 scope:** One secure element on the STM32 side only. Pi→cloud security is handled in software (see Security Architecture).

**Future upgrade (Phase 3+):** Advanced secure element or TPM on Pi for hardware-protected TLS client keys and fleet attestation. Requires a different part tier (e.g. ATECC608B-TNGTLS, OPTIGA Trust M) — not the same breakout used on STM32.

---

## Power Architecture

```mermaid
flowchart LR
  vehicleInput[Vehicle 12V or Battery] --> fuse[Fuse]
  fuse --> tvs[TVS Diode]
  tvs --> reverse[Reverse Polarity Protection]
  reverse --> buck5v[5V 5-6A Buck]
  reverse --> buck3v3[3.3V Low-Noise Buck]
  buck5v --> pi5[Pi 5 plus Camera NVMe]
  buck3v3 --> stm32[STM32 plus BNO085 GPS SE]
```

| Rail | Load | Budget |
|---|---|---|
| 5 V | Pi 5, camera, NVMe HAT, optional LTE | 5 A minimum, 6 A recommended |
| 3.3 V | STM32, BNO085, NEO-M8N, ATECC608A | Low-noise, sensor-dedicated |

Bench shortcut: USB-C PD supply for Pi 5 plus separate regulated sensor supply. Vehicle prototype uses the protected architecture above.

---

## Physical Integration

| Component | Rule |
|---|---|
| BNO085 | Rigid frame mount; not on loose enclosure wall |
| Camera | Forward-facing, known pitch, vibration-isolated, clean FOV |
| GPS antenna | Clear sky, away from Pi, DC/DC, LTE, camera cables |
| Pi 5 | Thermally managed, serviceable |
| STM32 | Near sensors; short I2C/UART traces; SWD accessible |
| Connectors | Locking (JST-GH or equivalent) for field testing |

---

## Data Flow

### Overview

```mermaid
flowchart LR
  subgraph sensors [Sensors]
    imu[BNO085]
    gps[NEO-M8N]
    cam[Camera]
  end

  subgraph mcu [STM32 plus ATECC608A]
    sample[Sample and Timestamp]
    frame[Frame and AEAD Tag]
  end

  subgraph pi [Raspberry Pi 5]
    receive[Verify Sensor Packets]
    vision[YOLO MiDaS RRI Fusion]
    fuse[Fuse Vision plus Motion plus GPS]
    log[NVMe Event Log]
    upload[mTLS Cloud Upload]
  end

  subgraph cloud [Cloud]
    verify[Verify mTLS and Event]
    store[Road Health Map]
  end

  imu --> sample
  gps --> sample
  sample --> frame
  frame -->|USB CDC| receive
  cam --> vision
  receive --> fuse
  vision --> fuse
  fuse --> log
  fuse --> upload
  upload --> verify
  verify --> store
```

### STM32 → Pi (Sensor Link)

```mermaid
sequenceDiagram
  participant IMU as BNO085
  participant GPS as NEO-M8N
  participant MCU as STM32
  participant SE as ATECC608A
  participant Pi as Raspberry Pi 5

  loop 100 Hz
    IMU->>MCU: Accel Gyro Orientation
    MCU->>MCU: Timestamp sample
  end

  loop 5 Hz
    GPS->>MCU: Lat Lon Speed Fix Quality
    MCU->>MCU: Timestamp sample
  end

  MCU->>SE: Request AEAD tag
  SE-->>MCU: Auth tag
  MCU->>Pi: USB CDC framed packet
  Pi->>Pi: Verify AEAD seq and timestamp
  Pi->>Pi: Update sensor state buffer
```

**Packet types:**

| Type | Rate | Payload |
|---|---|---|
| `IMU_SAMPLE` | 100 Hz | accel, gyro, orientation quaternion, calibration status |
| `GPS_FIX` | 1–10 Hz | lat, lon, alt, speed, course, fix type, HDOP, sats |
| `MCU_HEALTH` | 1 Hz | uptime, packet counters, auth failures, sensor status |
| `FAULT_EVENT` | On event | brownout, sensor reset, watchdog trip |

**Wire format:**

```
[0xAA 0x55][version:1][type:1][seq:4][timestamp_us:8][payload:N][AEAD_tag:16]
```

### Pi Internal (Vision + Sensor Fusion)

```mermaid
flowchart TB
  cam[Camera Frame] --> yolo[YOLO Detection]
  cam --> midas[MiDaS Depth]
  yolo --> geo[Geometry Check]
  midas --> geo
  yolo --> temp[Temporal Filter]
  geo --> temp
  temp --> rri[RRI Score]
  imuBuf[Latest IMU State] --> motionGate[Motion Gating]
  gpsBuf[Latest GPS Fix] --> geoTag[Geo Tagging]
  rri --> decision{RRI >= 0.75?}
  motionGate --> decision
  decision -->|Yes| event[Build Hazard Event]
  geoTag --> event
  decision -->|No| discard[Suppress]
```

### Pi → Cloud (Software Security — Phase 1)

```mermaid
sequenceDiagram
  participant App as Pi Application
  participant KeyStore as Software Key Store
  participant Store as NVMe
  participant Cloud as Cloud Ingest

  App->>App: Serialize hazard event
  App->>KeyStore: Load client cert and key
  App->>Store: Write event log
  App->>Cloud: TLS 1.3 mTLS POST
  Cloud->>Cloud: Verify client certificate
  Cloud->>Cloud: Validate event schema and seq
  Cloud->>Cloud: Accept or reject
```

**Phase 1 cloud security (software only):**

| Mechanism | Implementation |
|---|---|
| Transport | TLS 1.3 |
| Device authentication | mTLS with per-device X.509 client certificate |
| Key storage | Encrypted filesystem or OS keyring on Pi (software phase) |
| Replay protection | Monotonic event sequence numbers + timestamp window |
| At-rest logging | AES-256 file encryption on NVMe (software-managed key) |

**Example uplink event:**

```json
{
  "device_id": "vigia-a3f2c1",
  "sensor_hub_id": "hub-9b4e7d",
  "event_type": "hazard_detected",
  "timestamp_utc": "2026-06-09T14:32:01.042Z",
  "location": {
    "lat": 12.9716,
    "lon": 77.5946,
    "speed_mps": 8.3,
    "hdop": 1.2,
    "fix_quality": "3D"
  },
  "hazard": {
    "rri": 0.82,
    "confidence": 0.91,
    "geometry_score": 0.78,
    "temporal_score": 0.85,
    "bbox": [120, 200, 80, 60],
    "frame_id": 104892
  },
  "motion": {
    "accel_z": -0.12,
    "gyro_z": 0.03
  },
  "seq": 99102
}
```

Cloud trust in Phase 1 comes from **mTLS client certificate verification**, not from a Pi secure element signature.

---

## Security Architecture

### Trust Boundaries

```mermaid
flowchart TB
  subgraph hwTrust [Hardware Trust - Phase 1]
    se[ATECC608A on STM32]
    se -->|AEAD keys| mcuLink[MCU to Pi Link]
  end

  subgraph swTrust [Software Trust - Phase 1]
    mtls[mTLS Client Cert]
    tls[TLS 1.3 Transport]
    enc[NVMe Encryption at Rest]
    mtls --> cloudLink[Pi to Cloud Link]
    tls --> cloudLink
    enc --> localStore[Local Event Store]
  end

  mcuLink --> pi5[Raspberry Pi 5]
  pi5 --> cloudLink
```

### Layer Summary

| Layer | Link | Phase 1 Mechanism | Key Storage |
|---|---|---|---|
| L1 | MCU → Pi | AEAD + sequence + timestamp | ATECC608A on STM32 |
| L2 | Pi local | AES-256 at rest | Software key on NVMe/filesystem |
| L3 | Pi → Cloud | TLS 1.3 + mTLS | Software client cert/key on Pi |
| L4 | Cloud | Device registry PKI | Cloud backend |

### Threat Coverage

| Threat | Phase 1 Mitigation |
|---|---|
| Fake IMU/GPS injected on internal wire | AEAD on MCU→Pi; ATECC608 holds session key |
| Eavesdropping on cloud uplink | TLS 1.3 |
| Unauthorized device uploading to cloud | mTLS client certificate per device |
| Replay of old events | Monotonic seq + timestamp window |
| Stolen Pi reads cloud credentials | Encrypted key store (software); upgrade to HW SE in Phase 3 |
| GPS spoofing | Partial — IMU cross-check; dual-band GPS later |

### Phase 3 Upgrade Path (Pi Hardware Security)

When fleet scale or compliance requires it:

- Add advanced secure element or TPM on Pi (separate from STM32 ATECC608A breakout)
- Move TLS client private key and storage encryption key into hardware slot
- Optional ECDSA event signing in hardware
- Secure boot + signed firmware images

This is deferred to reduce Phase 1 BOM cost and integration complexity.

---

## Vision Pipeline (Existing Software)

```mermaid
flowchart LR
  subgraph core0 [Core 0 Coordinator]
    capture[Camera Capture]
    thermal[Thermal Monitor]
    dispatch[Frame Dispatch]
  end

  subgraph core1 [Core 1 Perception]
    yolo[YOLO26 INT8 every frame]
  end

  subgraph core2 [Core 2 Depth]
    midas[MiDaS FP32 stride-adaptive]
  end

  subgraph core3 [Core 3 Fusion]
    temporal[Temporal Filter]
    rri[RRI Fusion Engine]
    publish[publishResult]
  end

  capture --> dispatch
  dispatch --> yolo
  dispatch --> midas
  yolo --> temporal
  midas --> temporal
  temporal --> rri
  rri --> publish
```

| Parameter | Value |
|---|---|
| RRI weights | Detection 40%, geometry 35%, temporal 25% |
| Hazard threshold | RRI ≥ 0.75 |
| Full pipeline throughput | ~11 FPS (Pi 5, headless) |
| YOLO-only throughput | ~32 FPS (Pi 5) |

---

## Phased Rollout

### Phase 0 — Current

- Pi 5 vision pipeline operational
- Bench demo via VNC / UDP stream

### Phase 1 — Sensors + Link Security

- STM32 firmware: BNO085 @ 100 Hz, NEO-M8N @ 5 Hz
- ATECC608A on STM32: AEAD-protected MCU→Pi packets
- Pi sensor receiver + timestamp alignment with camera frames
- Software mTLS cloud uplink stub
- Motion gating prototype

### Phase 2 — Cloud Integration

- Device registry and PKI (per-device client certificates)
- TLS 1.3 mTLS production ingest endpoint
- NVMe encrypted event log
- Road Health Map aggregation (backend)

### Phase 3 — Fleet Hardening

- Advanced secure element or TPM on Pi (hardware cloud keys)
- Custom PCB with soldered ATECC608A
- Secure boot + signed firmware
- OTA updates

### Phase 4 — Production Features

- Accident detection (IMU spike + pre/post video clip)
- LTE uplink with offline buffer
- CAN bus evaluation (if vehicle market requires)

---

## Prototype BOM

| Item | Qty | Est. Cost | Notes |
|---|---|---|---|
| Raspberry Pi 5 8GB | 1 | $60–80 | Main compute |
| Pi Active Cooler | 1 | $5–10 | Required |
| Pi Camera Module 3 | 1 | $25–35 | Primary sensor |
| NVMe HAT + 128GB SSD | 1 | $25–40 | Field logging |
| STM32F411 Black Pill | 1 | $5–8 | Sensor hub |
| BNO085 breakout | 1 | $15–20 | IMU |
| NEO-M8N + antenna | 1 | $15–25 | GPS |
| ATECC608A breakout | 1 | $2–3 | STM32 only (Phase 1) |
| Automotive buck + protection | 1 | $10–20 | Power subsystem |
| Enclosure + connectors | 1 | $15–30 | Mechanical |
| **Total (prototype)** | | **~$175–260** | |

Fleet target with custom PCB: ~$80–120/unit at 1,000+ volume.

---

## Team Ownership

| Domain | Deliverables |
|---|---|
| Vision / ML | Extend `publishResult()`, motion gating, sensor-vision fusion |
| Firmware | STM32 BNO085/GPS drivers, USB CDC protocol, ATECC608 integration |
| Hardware | Power rails, mounting, connectors, I2C bus layout |
| Security | CryptoAuthLib on STM32, mTLS cert provisioning, device registry |
| Backend | Ingest API, mTLS termination, Road Health Map |

---

## Architecture Review

### Strengths at This Phase

1. **Split compute/sensor domains** — Standard approach for automotive edge; Pi cannot guarantee sensor timing under AI load.
2. **BNO085** — On-chip fusion avoids months of IMU tuning; proven in robotics.
3. **NEO-M8N** — Cheap, documented, upgrade path to M9/M10 for urban multipath.
4. **Single ATECC608A on STM32** — Protects the highest-risk link (sensor injection) at minimal BOM cost.
5. **Software mTLS for cloud** — Sufficient for prototype and early pilots; standard stack (OpenSSL), no Pi I2C bus contention with secure element.
6. **NVMe over SD** — Eliminates the primary Pi fleet failure mode.

### Accepted Gaps (Phase 1)

| Gap | Mitigation Timeline |
|---|---|
| Cloud keys in software on Pi | Phase 3 hardware secure element |
| Black Pill dev board | Phase 3 custom PCB |
| No secure boot | Phase 3 |
| GPS spoofing | Phase 2+ IMU dead reckoning |
| No OTA | Phase 3 |

### Verdict

**Appropriate for startup prototype with enterprise credibility targets.**

- Hardware secure element where it matters most (sensor authenticity on the internal wire)
- Software security where it is cheaper and faster to iterate (cloud mTLS, cert rotation, backend PKI)
- Clear upgrade path to hardware-protected cloud keys without redesigning data flows

The main execution risk is firmware and backend delivery speed, not the architecture itself.

---

## Glossary

| Term | Definition |
|---|---|
| AEAD | Authenticated Encryption with Associated Data |
| ATECC608A | Microchip secure element for hardware key storage |
| BNO085 | Bosch 9-DOF IMU with on-chip sensor fusion |
| mTLS | Mutual TLS — client and server both present certificates |
| NEO-M8N | u-blox M8 GPS module |
| RRI | Road Risk Index — fused hazard score, threshold 0.75 |
| USB CDC | USB serial device class — STM32 appears as `/dev/ttyACM0` |
