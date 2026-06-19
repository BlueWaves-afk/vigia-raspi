---
title: "EtHashInput (96-byte attestation struct)"
type: security
tags: [security, sensor-bridge]
source: firmware/src/main.c
related: ["[[ecdsa-signing]]", "[[atecc608a]]", "[[atecc608a-driver]]", "[[bno085-driver]]", "[[neo-m8n-driver]]", "[[sensor-bridge-node]]", "[[flow-sensor-bridge]]"]
updated: 2026-06-19
---

# EtHashInput

The 96-byte packed input that the Pico 2 firmware hashes (SHA-256) and signs via
[[ecdsa-signing]] on the [[atecc608a]] secure element. It binds the sensor event `E_t`
to the device identity so the cloud can verify provenance.

- **Fields:** `device_id` (from `atcab_read_serial_number()`), `timestamp`, IMU quaternion +
  linear accel (from [[bno085-driver]]), GPS fix (from [[neo-m8n-driver]]), sequence counter.
- **Hashing:** `atcab_sha()`; **Signing:** `atcab_sign()` → raw R‖S (64 bytes).
- **Transport:** packed into the `SignedEt` COBS frame and sent to the Pi via
  [[cobs-usb-cdc]], decoded by [[sensor-bridge-node]].
- **Anti-replay:** the ATECC-attested `sequence` is authoritative cloud-side, not the
  client-supplied value.

## Links
Produced by firmware → signed by [[ecdsa-signing]] → carried by [[flow-sensor-bridge]] → verified by [[mbedtls-verify]].
