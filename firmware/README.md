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

## Next step

When link + GPS work, move to separate VSYS power + **data-only USB** per [power-distribution.md](../docs/power-distribution.md), then add BNO085 + ATECC608A.
