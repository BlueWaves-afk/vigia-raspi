---
title: "AWS IoT MQTT (Paho)"
type: transport
tags: [transport, security]
source: .claude/design/05_anti_death_and_depin_contracts.md
related: ["[[anti-death-node]]", "[[sim7600]]", "[[tls-certs]]", "[[ecdsa-signing]]", "[[flow-anti-death]]"]
updated: 2026-06-19
---

# AWS IoT Core MQTT Transport

**Library:** Eclipse Paho MQTT C++ (`mqtt::async_client`)  
**Broker:** AWS IoT Core  
**Protocol:** MQTT over TLS 1.2 mutual authentication  
**QoS:** 1 (at-least-once, PUBACK required)  
**Status:** Phase 5 target (Phase 1 uses [[https-rest]] directly)

## Connection Configuration
```cpp
ssl_opts = mqtt::ssl_options_builder()
    .trust_store("/etc/vigia/ca_chain.pem")     // broker CA chain
    .key_store("/etc/vigia/device_cert.pem")    // device identity cert
    .private_key("/etc/vigia/device_key.pem")   // device private key
    .ssl_version(MQTT_SSL_VERSION_TLS_1_2)
    .verify(true)
    .finalize();

conn_opts = mqtt::connect_options_builder()
    .keep_alive_interval(20s)
    .connect_timeout(5s)
    .clean_session(false)           // QoS 1 session persistence across reboots
    .will("vigia/status/{device_id}", "OFFLINE", qos=1, retained=true)
    .finalize();
```

## Topics
| Topic | Direction | Use |
|---|---|---|
| `vigia/events/{device_id}/hazard` | Pi → Cloud | Emergency payload (MsgPack, ~440 KB) |
| `vigia/attest/{device_id}/hazard` | Pi → Cloud | Attested hazard event |
| `vigia/status/{device_id}` | Pi → Cloud | ONLINE/OFFLINE LWT status |

## Pre-Connection (startup, NOT in emergency window)
`mqtt_client_->connect(conn_opts_)->wait(15s)` — blocking at node startup. Keepalive PINGREQ/PINGRESP every 20 seconds keeps LTE bearer alive.

## Emergency Use
`do_mqtt_transmit()` calls `publish()` on pre-open TCP+TLS session — no handshake overhead during emergency window. QoS 1 session persistence: if PUBACK times out, broker retransmits on next reboot+reconnect.

## MsgPack Payload (~440 KB)
`frame_metadata` + `spatial_latent` (S_t bin 400 KB) + `signed_et` + `hazard_event`. At 2 Mbps UL: ~1.8 s transmit time.

## Links
- Used by: [[anti-death-node]] (emergency transmit)
- LTE transport: [[sim7600]] (ECM `usb0`)
- Certs: [[tls-certs]]
- Signs payload: [[ecdsa-signing]] (ATECC608A), [[ed25519-signing]] (libsodium)
- Flow: [[flow-anti-death]]
