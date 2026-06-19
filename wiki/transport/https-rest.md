---
title: "HTTPS REST (libcurl → AWS API Gateway)"
type: transport
tags: [transport, security]
source: vigia_ws/src/vigia_edge_node/src/anti_death_node.cpp
related: ["[[anti-death-node]]", "[[ed25519-signing]]", "[[tls-certs]]", "[[flow-anti-death]]"]
updated: 2026-06-19
---

# HTTPS REST Transport — Phase 1 Emergency Uplink

**Library:** libcurl  
**Endpoint:** `https://sq2ri2n51g.execute-api.us-east-1.amazonaws.com/prod/telemetry` (AWS API Gateway, prod stage)  
**Method:** POST  
**Auth:** Ed25519 detached signature in request header (libsodium, see [[ed25519-signing]])  
**Status:** Phase 1 implementation — written and wired

## Why HTTPS REST (not MQTT) in Phase 1
Phase 5 MQTT via [[aws-iot-mqtt]] + [[sim7600]] was deferred. HTTPS REST via libcurl works with existing AWS API Gateway backend and requires no MQTT broker setup.

## libcurl Configuration (initialized at node startup)
```cpp
curl_easy_setopt(curl_, CURLOPT_URL, aws_telemetry_url_.c_str());
curl_easy_setopt(curl_, CURLOPT_POST, 1L);
curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYPEER, 1L);
curl_easy_setopt(curl_, CURLOPT_CONNECTTIMEOUT, 8L);  // T+3→T+11s budget
curl_easy_setopt(curl_, CURLOPT_TIMEOUT, 12L);
```
Handle initialized at startup — only `curl_easy_perform()` called during emergency.

## Payload
JSON body containing:
- `device_id` (UUID)
- `timestamp_us`
- `et_hash`, `ecdsa_sig` (from [[atecc608a]]/[[ecdsa-signing]])
- `rri_score`, `iss_score`
- `spatial_latent` (base64 or array)
- Ed25519 signature header

## Response
Expects HTTP 202 Accepted. Budget: ≤13s from UPS GPIO assert to PUBACK/response.

## Links
- Used by: [[anti-death-node]] (Phase 1 emergency transmit)
- Signs with: [[ed25519-signing]] (libsodium, 32-byte key at `/etc/vigia/device_ed25519.key`)
- SSL: [[tls-certs]]
- Flow: [[flow-anti-death]]
