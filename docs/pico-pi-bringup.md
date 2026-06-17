# Pi ↔ Pico 2 Link Bring-Up

First milestone: Pico 2 enumerates as USB serial on the Pi and sends `VIGIA_PING` once per second.

No IMU or GPS required for this step.

---

## What you need

| Item | Notes |
|------|-------|
| Raspberry Pi 5 | Running Linux, SSH or local keyboard |
| Raspberry Pi Pico 2 | Flashed with `vigia_pico_hello.uf2` |
| USB cable | micro-USB or USB-C depending on Pico 2 board |

---

## Step 1 — Flash the Pico 2

Build on Mac or Linux (see [firmware/README.md](../firmware/README.md)), then:

1. Hold **BOOTSEL** on the Pico 2.
2. Plug into your computer.
3. Release BOOTSEL.
4. Copy `firmware/build/vigia_pico_hello.uf2` to the USB drive that appears.
5. Onboard LED should blink once per second after reboot.

---

## Step 2 — Connect Pico to Pi

### Option A — Simplest bench test (recommended first)

```
Pico 2 USB  →  Pi 5 USB port
```

Pi powers the Pico. No extra wiring.

### Option B — Production-style (later)

```
5 V buck  →  Pi 5 + Pico VSYS (pin 39)
Pico USB    →  Pi USB with data-only cable (5 V wire cut or power blocker)
```

Use Option B only after Option A passes.

---

## Step 3 — Check USB enumeration on the Pi

```bash
ls -l /dev/ttyACM*
lsusb | grep -i "2e8a\|Raspberry"
dmesg | tail -20
```

**Pass:** `/dev/ttyACM0` exists within ~5 s of plugging in.

If nothing appears:

- Try another USB port on the Pi.
- Reflash the UF2 (BOOTSEL method).
- Confirm the cable carries **data**, not charge-only.

---

## Step 4 — Read data

### Raw

```bash
sudo apt install -y python3-serial
cat /dev/ttyACM0
```

Expected (1 Hz):

```
VIGIA_PING seq=0 uptime_ms=2500 boot_ms=1200
VIGIA_PING seq=1 uptime_ms=3500 boot_ms=1200
```

Press Ctrl+C to stop. If `cat` shows nothing, wait 2–3 s after plug-in (USB enumerate delay).

### Monitor tool

From the repo on the Pi:

```bash
python3 tools/pico_bridge_monitor.py --duration 15
```

**Pass:** ≥ 5 messages, ~1 Hz, zero sequence gaps, prints `[PASS]`.

---

## Step 5 — Permissions (optional)

Avoid `sudo` for serial:

```bash
sudo usermod -aG dialout $USER
# log out and back in
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| No `/dev/ttyACM0` | Reflash UF2; try different USB port/cable |
| Permission denied | `sudo usermod -aG dialout $USER` |
| Garbled text | Wrong baud — use 115200 (default for Pico stdio USB) |
| `cat` hangs with no data | Unplug/replug; run `dmesg \| tail`; confirm UF2 flashed |
| Device is `/dev/ttyACM1` | Use `--port /dev/ttyACM1` |
| LED blinks but no serial | Rebuild with `pico_enable_stdio_usb(1)` — already set in hello firmware |

---

## After this works

1. Confirm stable 30+ minutes with `pico_bridge_monitor.py --duration 1800` (optional).
2. Switch to VSYS + data-only USB if using a shared 5 V buck.
3. Wire BNO085 on SPI0 — next firmware increment.
4. Wire GPS on UART — after IMU is stable.

---

## Quick checklist

- [ ] `vigia_pico_hello.uf2` flashed; LED blinks 1 Hz
- [ ] `/dev/ttyACM0` present on Pi
- [ ] `VIGIA_PING` visible in `cat` or monitor
- [ ] `pico_bridge_monitor.py` reports `[PASS]`
