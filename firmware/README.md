# Pico 2 firmware — Pi link test

Minimal USB CDC heartbeat for verifying Pi ↔ Pico 2 before sensors are wired.

## Prerequisites

On the machine used to **build** firmware (Mac or Linux):

```bash
# Pico SDK
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git submodule update --init && cd ..

# ARM GCC toolchain (macOS example)
brew install cmake gcc-arm-embedded

export PICO_SDK_PATH=/path/to/pico-sdk
```

## Build

```bash
cd firmware
cmake -B build -DPICO_BOARD=pico2
cmake --build build
```

Output: `build/vigia_pico_hello.uf2`

## Flash

1. Hold **BOOTSEL** on the Pico 2.
2. Plug USB into your laptop (or Pi).
3. Release BOOTSEL — a drive named **RP2350** (or **RPI-RP2**) appears.
4. Copy `build/vigia_pico_hello.uf2` onto that drive.
5. Pico reboots automatically.

## Bench wiring (simplest — Pi powers Pico)

For the **first link test only**:

```
Pico 2 micro-USB  →  Pi 5 USB port
```

No sensors required. Pico is powered from the Pi over USB.

## Verify on the Pi

```bash
# Device present?
ls -l /dev/ttyACM*

# Raw stream (Ctrl+C to stop)
cat /dev/ttyACM0

# Or use the monitor tool
python3 tools/pico_bridge_monitor.py
```

Expected output once per second:

```
VIGIA_PING seq=0 uptime_ms=2500 boot_ms=1234
VIGIA_PING seq=1 uptime_ms=3500 boot_ms=1234
```

## NEO-M8N GPS (UART1)

Wire the u-blox NEO-M8N to the Pico 2 header:

| M8N pin | Pico 2 |
|---------|--------|
| TX      | GP9 (UART1 RX) |
| RX      | GP8 (UART1 TX) |
| VCC     | 3V3 (OUT) |
| GND     | GND |

Rebuild and reflash `vigia_pico_hello.uf2`. On the Pi:

```bash
python3 tools/pico_gps_monitor.py --duration 30
# or JSON: python3 tools/pico_gps_monitor.py --json
```

Expected @ 1 Hz once UART sees NAV-PVT frames:

```
VIGIA_GPS seq=0 lat=37.1234567 lon=-122.1234567 speed_ms=0.00 fix_type=3 satellites=12 hdop=0.85 valid=1
```

Without GPS wired you still get `VIGIA_PING` (link test).

## BNO085 IMU (SPI0)

Wire the BNO085 (BN-085 breakout) to the Pico 2 header:

| BNO085 pin | Pico 2 GPIO | Pico pin | Notes |
|------------|-------------|----------|-------|
| **SCL** | GP18 | Pin 24 | SPI SCK |
| **AD0** | GP19 | Pin 25 | SPI MOSI |
| **SDA** | GP16 | Pin 21 | SPI MISO |
| **CS** | GP17 | Pin 22 | Chip select (active LOW) |
| **INT** | GP20 | Pin 26 | Interrupt (active LOW) |
| **RST** | GP21 | Pin 27 | Reset (active LOW) |
| **PS0** | **GP22** | **Pin 29** | **WAKE — must wire to Pico, not only 3.3 V** |
| **PS1** | 3.3 V | — | Solder jumper or wire to 3.3 V (SPI mode) |
| **VCC** | 3.3 V | Pin 36 | Sensor rail |
| **GND** | GND | Pin 38 | Common ground |

> **PS0 is critical for SPI.** After reset it becomes the WAKE line. Remove any PS0→3.3 V jumper and wire **PS0 → GP22** instead. **PS1** must be HIGH at reset (jumper to 3.3 V).

If init fails, check `imu_stage` in `VIGIA_PING`:

| `imu_stage` | Meaning |
|-------------|---------|
| 0 | Not started |
| 1 | Boot timeout — no INT after reset (check PS1=HIGH, wiring) |
| 2 | Rotation vector enable failed |
| 3 | Linear accel enable failed |
| 4 | Ready |
| 5 | No sensor data within 3 s after enable (check PS0→GP22, PS1→3.3 V) |

> Power BNO085 from a **dedicated 3.3 V sensor rail** when GPS + IMU + ATECC are all populated — see [power-distribution.md](../docs/power-distribution.md).

Rebuild and reflash. On the Pi you should see `VIGIA_IMU` lines @ 10 Hz (bench debug) alongside GPS:

```
VIGIA_IMU seq=0 qw=0.9980 qx=0.0120 qy=-0.0030 qz=0.0550 ax=0.010 ay=-0.020 az=0.150 cal=3 valid=1 qnorm=1.0000
```

Bench checks (Milestone 1):

- `imu_ready=1` in `VIGIA_PING` when init succeeded
- Quaternion norm `qnorm` ≈ 1.0 ± 0.02 when stationary
- Rotate board 90° — quaternion components change predictably
- GPS lines still print (`VIGIA_GPS` unchanged)

Disable IMU USB debug: rebuild with `-DVIGIA_IMU_DEBUG_USB=0` in CMake.

## ATECC608P Secure Element (I2C1 — Phase 2)

4-pin breakout (VCC, GND, SCL, SDA only). ADDR is hardwired on the PCB (usually **0x60**).

| Breakout pin | Pico 2 GPIO | Notes |
|--------------|-------------|-------|
| SDA | **GP2** | I2C1 |
| SCL | **GP3** | I2C1 |
| VCC | 3.3 V sensor rail | Shared with BNO085 + GPS — see [power-distribution.md](../docs/power-distribution.md) |
| GND | Common ground | Star point at regulator |

**Pull-ups:** Most breakouts include onboard SDA/SCL pull-ups (2.2–10 kΩ). If your PCB has them, do **not** add external 4.7 kΩ resistors. Pico `gpio_pull_up()` in the driver is supplementary only (~50 kΩ).

**If init fails** (LED blinks 3×): I2C-scan GP2/GP3. Trust&GO parts may use `0x35`/`0x36` — change `VIGIA_ATCA_ADDR` in `src/atecc608a_driver.c`.

### Phase 2 stub build (COBS pipeline, zero signatures — no SE required)

```bash
cd firmware
cmake -B build-phase2-stub -DPICO_BOARD=pico2 -DVIGIA_BUILD_PHASE2_STUB=ON
cmake --build build-phase2-stub
# Flash: build-phase2-stub/vigia_pico_phase2_stub.uf2
```

On the Pi, validate COBS output:

```bash
python3 tools/pico_packet_monitor.py --port /dev/ttyACM0 --duration 30
```

### Phase 2 live build (real ECDSA — SE wired)

```bash
# One-time: clone cryptoauthlib (or run scripts/setup_cryptoauthlib.sh)
git clone https://github.com/MicrochipTech/cryptoauthlib.git firmware/third_party/cryptoauthlib

cd firmware
cmake -B build-phase2-live -DPICO_BOARD=pico2 \
  -DVIGIA_BUILD_PHASE2_LIVE=ON \
  -DCRYPTOAUTHLIB_DIR=third_party/cryptoauthlib
cmake --build build-phase2-live
# Flash: build-phase2-live/vigia_pico_phase2_live.uf2
```

Provision slot-0 key and export pubkey:

```bash
python3 tools/atecc_provision.py --port /dev/ttyACM0 --device-id vigia-pico-001
```

Bench validation:

```bash
./scripts/bench_phase2_validate.sh --port /dev/ttyACM0
```

## Next step

When link + GPS + IMU work, move to separate VSYS power + **data-only USB** per [power-distribution.md](../docs/power-distribution.md). Phase 2 COBS+signing firmware replaces text `VIGIA_GPS` output when `VIGIA_PHASE2=1`.
