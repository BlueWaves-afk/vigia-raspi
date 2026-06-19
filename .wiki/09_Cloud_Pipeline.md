# Cloud Pipeline (AWS — M12)

Since **M12** the backend is fully serverless on AWS (repo: `vigia-amazon`, CDK stack
`VigiaStack`). The self-hosted FastAPI + Mosquitto + PostgreSQL stack described in the
older [[05_Event_Pipeline]] was **deleted**. Two ingest paths converge on one
`HazardsTable`, which a DynamoDB-stream pipe fans out to verification.

## End-to-end flow

```
                 mTLS MQTT QoS-1
 Pi hazard_uplink ───────────────▶ AWS IoT Core ──(Topic Rule vigia_hazard_attest)──▶ AttestationFn (Lambda)
   (MsgPack, ECDSA)                  topic: vigia/attest/{device_id}/hazard               │ verify chain
                                                                                          ▼
 Phone app ──HTTPS POST /telemetry──▶ ValidatorFn (Lambda) ──Ed25519 verify──▶ ┌──────────────────┐
   (Ed25519 signed)                                                            │   HazardsTable    │ (status=PENDING)
                                                                               └─────────┬────────┘
                                                                          DynamoDB stream │ (EventBridge Pipe, INSERT)
                                                                                          ▼
                                                                              OrchestratorFn (Lambda)
                                                                          2% → Bedrock Nova VLM + ReAct Agent
                                                                          98% → ONNX-confidence fast path
                                                                                          │ VERIFIED
                                                       ┌──────────────────────────────────┼───────────────────┐
                                                       ▼                                   ▼                   ▼
                                              tryCreditReward (atomic)            HazardsTable status=VERIFIED  Solana submit
                                              RewardsLedger + dedup lock                   │ (stream MODIFY pipe)
                                                                                          ▼
                                                                                 Maintenance SQS (fan-out)
```

## Lambdas

| Function | Trigger | Role |
|----------|---------|------|
| `AttestationFn` | IoT Rule `vigia_hazard_attest` | MsgPack decode → EtHash verify → ECDSA P-256 → H3 res-10 dedup → `HazardsTable` upsert + `AttestationLog`. See [[10_Cloud_Security_Model]] |
| `ValidatorFn` | API GW `POST /telemetry` | Ed25519 verify → registry/blacklist check → `HazardsTable` PENDING + S3 frame |
| `OrchestratorFn` | `HazardsTable` stream (INSERT) via EventBridge Pipe | 2% VLM (`VLM_SAMPLE_RATE`) + ReAct Agent; 98% ONNX fast path; `tryCreditReward`; Solana settle |
| `SlashNodeFn` | async from Orchestrator | on-chain slash + `blacklisted=true` in registry |
| `RegisterDeviceFn` | API GW `POST /register-device` | Ed25519 proof-of-possession → registry |
| `ClaimDeviceFn` | API GW `POST /claim-device` | 1:1 device↔wallet binding |
| `RewardsBalanceFn` | API GW `GET /rewards-balance` | wallet balance read |

## DynamoDB tables

| Table | Key | Purpose |
|-------|-----|---------|
| `HazardsTable` | `geohash` + `timestamp` (GSI `h3-hazardtype-index`, `status-timestamp-index`) | hazard records; stream source |
| `VigiaPiDeviceRegistry` | `device_id` | Pi `cert_pem` + `last_seq` anti-replay watermark (RETAIN) |
| `VigiaDeviceBindings` | `device_id` (GSI `wallet-pubkey-index`) | 1:1 hardware↔wallet (RETAIN) |
| `RewardsLedger` | `wallet_address` | $VIGIA balances |
| Ledger (`ContributorGeohashIndex`) | `ledgerId`+`timestamp` | append-only reward ledger; per-(wallet,geohash) dedup |
| Cooldown | `cooldownKey` | `proc#…` processing dedup + `rwd#…` 30-day reward lock |
| `AttestationLog` | `pk` (`device_id#seq`) | verified-event audit log |

## Resilience
Orchestrator and slash-node have SQS **DLQs** (2 retries). The verification fan-out
to maintenance SQS filters DynamoDB-stream **`MODIFY`** events with `status=VERIFIED`
(hazards are inserted PENDING, promoted on update).

## Related
[[00_Index]] · [[08_ROS2_Edge_Nodes]] · [[10_Cloud_Security_Model]] · [[05_Event_Pipeline]] · [[03_Telemetry_Signing]]

#tags: #aws #serverless #depin #vigia-stack
