---
title: "mbedTLS ECDSA Verify (Pi-side)"
type: security
tags: [security, sensor-bridge]
source: vigia_ws/src/vigia_edge_node/src/sensor_bridge_node.cpp
related: ["[[sensor-bridge-node]]", "[[ecdsa-signing]]", "[[et-hash-input]]", "[[atecc608a]]", "[[flow-sensor-bridge]]"]
updated: 2026-06-19
---

# mbedTLS ECDSA Verify

Pi-side verification of the Pico 2's secp256r1 signature on each [[et-hash-input]] before
the [[sensor-bridge-node]] trusts a `SignedEt` frame. Loads the secure element public key
from `/etc/vigia/atecc_pubkey.bin` at startup and calls `VigiaIdentityKey::verify_peer()`
inside `process_cobs_frame`.

- **Returns `false`** for all-zero stub signatures (i.e. when [[atecc608a]] is not yet
  provisioned / firmware is in `VIGIA_PHASE2_STUB=1` mode).
- **`sig_valid`** result is propagated downstream into the MsgPack hazard payload so the
  cloud attestation Lambda can make the same trust decision.

## Links
Counterpart of [[ecdsa-signing]] · verifies [[et-hash-input]] · gates [[sensor-bridge-node]] → [[flow-sensor-bridge]].
