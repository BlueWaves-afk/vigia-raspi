# Cloud Security Model

Covers the trust chain and economic-integrity controls of the [[09_Cloud_Pipeline]].
Hardened in **session 5** (2026-06-19) following a full pipeline review; all controls
below are deployed (`VigiaStack` → `UPDATE_COMPLETE`).

## Two trust tiers

| Path | Signer | Strength | Notes |
|------|--------|----------|-------|
| Hardware attest (`AttestationFn`) | ATECC608A P-256 (per-device) | Strong | Decorative until ATECC provisioned — `sig_valid` is currently zero-stubbed (GAP 2.8) |
| Mobile telemetry (`ValidatorFn`) | Ed25519 wallet key | Weaker | Working path today; frame is not signature-covered (residual H2) |

## Hardware attestation chain (`AttestationFn`)

```
MsgPack decode → rebuild EtHashInput (96-byte LE struct, byte-identical to
03_pico2_firmware_contracts §6.3.2) → SHA-256 == received et_hash
   → ECDSA P-256 verify (prehash:false, lowS:false; raw R‖S from ATECC)
   → advance anti-replay sequence  ← ONLY after signature is proven valid (S.5)
   → H3 res-10 geo-dedup → HazardsTable upsert → AttestationLog
```

> [!WARNING]
> **Order matters.** The sequence watermark is advanced *after* signature
> verification. Advancing first (the pre-session-5 bug) let a forged packet poison
> the watermark and lock out the device's later legitimate messages.

## Anti-replay
`AttestationFn` uses an atomic DynamoDB conditional update on `VigiaPiDeviceRegistry`:
`attribute_not_exists(last_seq) OR last_seq < :seq`. The sequence is ATECC-attested,
not client-supplied. `ValidatorFn` enforces a ±10 min timestamp freshness window.

## Economic integrity (session 5)

| Control | Mechanism |
|---------|-----------|
| Reward dedup | One reward per `(wallet, geohash)` per 30 days — `rwd#…` cooldown lock |
| Atomicity | `tryCreditReward` = single `TransactWriteCommand` (dedup-lock + balance + ledger); no read-then-write race |
| Sybil resistance | `register-device` requires Ed25519 proof-of-possession over `VIGIA-REGISTER:<pubkey>` |
| Slashing enforced | `ValidatorFn` rejects `blacklisted=true` (403); spoof → `SlashNodeFn` on-chain + blacklist |
| Probabilistic verify | 2% VLM sample is sound *because* dedup + slashing make cheating −EV |
| Fail-closed VLM | Bedrock output parsed defensively; NaN/garbage → no reward |

> [!NOTE]
> The Android client must sign the registration payload — `WalletRepositoryImpl`
> was updated to match (session 5). **Requires an APK rebuild** or onboarding 401s.

## Residual risks
- ATECC not provisioned → hardware path non-functional (GAP 2.8, physical).
- Mobile frame not covered by signature (H2).
- Legacy duplicate pipe `vigia-hazards-to-orchestrator` still live (S.9 — manual delete).

## Related
[[00_Index]] · [[09_Cloud_Pipeline]] · [[03_Telemetry_Signing]] · [[08_ROS2_Edge_Nodes]]

#tags: #security #depin #attestation #vigia-stack
