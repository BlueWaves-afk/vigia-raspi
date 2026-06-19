---
title: "Raspberry Pi Pico 2 (RPU)"
type: hardware
tags: [hardware, firmware]
source: .claude/design/03_pico2_firmware_contracts.md
related: ["[[raspberry-pi-5]]", "[[atecc608a]]", "[[bno085]]", "[[neo-m8n]]", "[[bno085-driver]]", "[[neo-m8n-driver]]", "[[atecc608a-driver]]", "[[cobs-tx-driver]]", "[[no-heap]]", "[[sensor-bridge-node]]", "[[cobs-usb-cdc]]"]
updated: 2026-06-19
---

# Raspberry Pi Pico 2 / RP2350 (Remote Processing Unit)

**SoC:** RP2350  
**CPU:** ARM Cortex-M33 @ 150 MHz (single-core firmware — Core 1 parked)  
**SRAM:** 520 KiB  
**Flash:** 4 MiB onboard QSPI (firmware uses ~150 KB = 4%)  
**USB:** TinyUSB CDC (native device controller)  
**Crystal:** 12 MHz onboard

## Role
Bare-metal sensor aggregation and cryptographic signing RPU. Aggregates [[bno085]] IMU (100 Hz) + [[neo-m8n]] GPS (1 Hz) + [[atecc608a]] ECDSA signing, produces COBS-framed `SignedEtPacket` over USB-CDC to [[raspberry-pi-5]].

## Execution Model
**Interrupt-driven DMA super-loop** — NO RTOS. Core 0 runs infinite loop with `__wfi()`. Core 1 permanently parked.

## Pin Assignment Summary
| Peripheral | Pins | Config |
|---|---|---|
| SPI0 (BNO085) | GP18/19/16/17 (SCK/MOSI/MISO/CS) | 3.0 MHz, Mode 3 |
| BNO085 INT | GP20 (IRQ, active LOW) | Falling edge |
| BNO085 RST | GP21 (active LOW) | Output |
| BNO085 WAKE | GP22 (PS0, required for SPI) | Output |
| UART1 (GPS) | GP8/GP9 (TX/RX) | 9600 baud |
| I2C1 (ATECC608A) | GP2/GP3 (SDA/SCL) | 400 kHz, addr 0x60 |
| USB | D+/D− | TinyUSB CDC |
| LED_STATUS | GP25 | Onboard |
| LED_ERROR | GP22 (external) | Error indicator |

## DMA Channels
| Channel | Direction | Peripheral |
|---|---|---|
| 0 | Periph→Mem | SPI0 RX (BNO085 receive) |
| 1 | Mem→Periph | SPI0 TX (zeros) |
| 2 | Periph→Mem | UART1 RX (GPS circular) |

## Static Memory Budget (520 KiB)
- Pico SDK + TinyUSB: ~40 KB
- .bss (all static buffers): ~20 KB
- Main stack: 8 KB
- Flash: ~150 KB / 4096 KB (4%)

## Build
```bash
cmake -B build -DPICO_BOARD=pico2 && cmake --build build
cp build/vigia_pico2.uf2 /media/RP2350/  # BOOTSEL drag-and-drop
```

## Firmware Drivers
- [[bno085-driver]] — SPI0 DMA SHTP, 100 Hz
- [[neo-m8n-driver]] — UART1 ring buffer UBX, 1 Hz
- [[atecc608a-driver]] — I2C1 cryptoauthlib (Phase 2, SE not yet wired)
- [[cobs-tx-driver]] — COBS encoder + USB-CDC
- [[no-heap]] — link-time allocation ban

## Links
- Connected to: [[raspberry-pi-5]] (USB-CDC), [[atecc608a]] (I2C1), [[bno085]] (SPI0), [[neo-m8n]] (UART1)
- Transport: [[cobs-usb-cdc]] → [[sensor-bridge-node]]
