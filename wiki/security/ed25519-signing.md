---
title: "Ed25519 Signing (AntiDeathNode uplink)"
type: security
tags: [security, anti-death]
source: vigia_ws/src/vigia_edge_node/src/anti_death_node.cpp
related: ["[[anti-death-node]]", "[[https-rest]]", "[[aws-iot-mqtt]]", "[[flow-anti-death]]", "[[ecdsa-signing]]", "[[tls-certs]]"]
updated: 2026-06-19
---

# Ed25519 Signing

libsodium Ed25519 device signature applied by the [[anti-death-node]] on the emergency
uplink payload during a power-loss event. Distinct from the hardware [[ecdsa-signing]]
done on the Pico 2 — this is the Pi-side software signature over the serialized snapshot
that goes out via [[https-rest]] (and is the unsigned-path gap when Paho/[[aws-iot-mqtt]]
is unavailable).

- **Key:** `device_ed25519.key` loaded at node init.
- **Scope:** the anti-death JSON/MsgPack payload assembled in the 15-second window.
- **Status:** key path is read; the legacy curl HTTPS transmit path currently sends the
  payload *unsigned* (low-priority gap — only the MQTT mTLS path via [[tls-certs]] is
  cryptographically protected end-to-end).

## Links
Upstream: [[anti-death-node]] · Downstream: [[https-rest]], [[aws-iot-mqtt]] · Related: [[ecdsa-signing]], [[et-hash-input]], [[flow-anti-death]]
