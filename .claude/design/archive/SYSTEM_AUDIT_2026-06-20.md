# VIGIA ADAS-DePIN — System Security & Architecture Audit
**Date:** 2026-06-20  **Auditor:** Principal Security + Systems review
**Scope:** vigia-raspi (edge ROS2 + Pico2 firmware), vigia-android (Kotlin), vigia-amazon (AWS CDK + live account `203800220566`, us-east-1)
**Access:** Live AWS read-only confirmed (`arn:aws:iam::203800220566:user/vigia-developer`).

> Method: wiki vaults used as a map, then every claim verified against real source + live AWS resources. All findings cite `file:line` or a concrete AWS resource. No source was modified; no AWS mutation was performed.

---

## 1. Executive Summary

| Repo | Posture | Notes |
|---|---|---|
| vigia-amazon (cloud) | **POOR — ships money/secret/data leaks** | Every API route is `auth=NONE`, security rests entirely on per-Lambda signature checks; several of those checks are missing, replayable, or fail-open. The DePIN economic core is farmable. |
| vigia-android (mobile) | **FAIR** | Crypto wiring is reasonable (TEE Ed25519). Inherits the cross-endpoint replay design flaw because it reuses one proof string for read and payout. |
| vigia-raspi (edge) | **GOOD with one fail-open flag** | ROS2 line is the production path; firmware/COBS/anti-replay are solid. Legacy `src/*.cpp` still has a stub-signature bypass flag (default off). |

**Single most important finding (P0):** The reward economy is **trivially farmable for real money**. 98% of hazards are verified by a deterministic ONNX-confidence fast path (`orchestrator/index.ts:207-222`) with no spoof check; the attacker controls the wallet (open self-registration, `register-device`), the geohash, and the confidence value; rewards are only deduped per `(wallet, geohash)`; and `/stripe/payout-session` never decrements `pending_balance` and is reachable unauthenticated. An attacker can mint unlimited `pending_balance` across geohashes and cash it out repeatedly to fiat.

---

## 2. CRITICAL Security Leaks & Errors (P0)

### P0-1 — Every API Gateway route is unauthenticated (`auth=NONE`, no API key)
**Evidence (live AWS):** all routes on the three core REST APIs resolve to `authorizationType=NONE, apiKeyRequired=false`:
- Telemetry API `sq2ri2n51g`: `/telemetry`, `/register-device`, `/claim-device`, `/stripe/{onboard,payout,financial}-session`, `/sarvam-proxy/{stt,tts}`, `/rewards`(read), `/traces`, `/ledger`, `/hazards`, `/verify-hazard-sync`.
- Innovation API `p4qc9upgsf`: `/rewards-balance`, `/maintenance/report`, `/economic/metrics`, `/routing-agent/branch`, `/agent-traces/stream`.
- Session API `eepqy4yku7`: full `/sessions` CRUD incl. `DELETE /sessions/{id}`, `/places/search`, `/geohash/resolve`.

**Impact:** No platform-level throttle/authz; every protection is hand-rolled per Lambda. Where a Lambda forgets the check (P0-2, P0-5, P0-6) the endpoint is fully open. CORS is `*` on all of them.
**Fix:** Add a Lambda/Cognito/IAM authorizer or at minimum a per-API usage plan + API key for cost control. For signature-gated routes, keep the app-layer check but add WAF rate-limiting and `apiKeyRequired` to bound abuse. Use `cdk-nag` (AwsSolutions-APIG4/COG4) to fail the build on unauthorized routes.

### P0-2 — Sarvam API-key proxy is open to the anonymous internet (key theft / quota drain)
**Evidence:** `sarvam-proxy/index.ts:13` doc-comment claims *"Caller must be authenticated (Cognito JWT via Authorization header)"* — but the handler **never reads `Authorization`** (`sarvam-proxy/index.ts:32-123`) and the route is `auth=NONE`. The secret `vigia/sarvam-api-key` is injected as env var and used directly (`:55, :103`).
**Exploit:** `curl -X POST https://<telemetry-api>/sarvam-proxy/tts -d '{"text":"...","target_language_code":"hi-IN"}'` → free unlimited STT/TTS billed to VIGIA's Sarvam account.
**Fix:** Enforce the JWT (Cognito authorizer on the route) or require the wallet-ownership proof; add reserved concurrency + a usage plan.

### P0-3 — Cross-endpoint signature replay: balance proof is accepted for money movement
**Evidence:** Both read (`rewards-balance/index.ts:52`) and payout (`stripe-payout/index.ts:61`) verify the **same** message `VIGIA-BALANCE:${wallet}:${tsMs}`. Android signs this identical string for balance polling every 60s (`feature/copilot/.../CopilotViewModel.kt:388,408`; `core/wallet/.../WalletRepositoryImpl.kt:65`) and for Stripe. The freshness window is 5 min and the proof is **not single-use** (no nonce consumption).
**Exploit:** Any party that observes one balance proof (a logging sidecar, a malicious analytics endpoint, a replayed request within 5 min) can call `/stripe/payout-session` and `/stripe/onboard-session` as the victim. There is no domain separation between "read my balance" and "send me money."
**Fix:** Domain-separate the message per action (`VIGIA-PAYOUT:<wallet>:<amount>:<ts>:<nonce>`), make payout proofs single-use by atomically consuming a server-issued nonce in DynamoDB, and bind the signed payload to the amount.

### P0-4 — Stripe payout never debits the balance + no idempotency → repeatable cash-out
**Evidence:** `stripe-payout/index.ts:106-137` reads `pending_balance`, creates a PaymentIntent for the full amount, and returns — **no `UpdateCommand` to decrement `pending_balance`** anywhere in the codebase (`grep pending_balance` shows only `ADD` in orchestrator and `GET` in payout). No `stripe.paymentIntents.create(..., {idempotencyKey})`. No Stripe webhook handler exists (`grep -i webhook` → none).
**Exploit:** Earn balance once → call `/payout-session` N times → N PaymentIntents for the same balance; or retry the same intent. Real fiat leaves per call.
**Fix:** Wrap payout in a DynamoDB conditional `UpdateCommand` that atomically zeroes/decrements `pending_balance` (condition `pending_balance >= :amt`) **before** creating the intent; pass a deterministic Stripe `idempotencyKey` (e.g. `payout#<wallet>#<nonce>`); add a webhook to reconcile `payment_intent.succeeded`/`failed` and roll back on failure.

### P0-5 — Reward economy is farmable (the DePIN attestation guarantee is broken)
**Evidence:**
- Open self-registration: `register-device/index.ts` only requires proof-of-possession of a key *the attacker generated* — it does not bind to attested hardware. Unlimited Sybil wallets.
- Mobile ingest (`validator/index.ts`) signs `driverWalletAddress = publicKey` (`:108`) — the wallet is self-asserted, and the only gate is "present in registry" (`:80`). No hardware attestation, **no H3/geo dedup** on this path (dedup exists only in `attestation/index.ts:170`).
- 98% fast path credits a full `ONE_TOKEN` reward on `confidence >= 0.65` with **no VLM/agent check** (`orchestrator/index.ts:207-222`). Confidence is an attacker-supplied field in the signed payload.
- Reward dedup is per `(wallet, geohash)` only (`orchestrator/index.ts:144`). Attacker iterates geohashes → unlimited tokens. Slashing only fires on the 2% VLM sample with conf < 0.1 (`:278`), so expected loss to farming is ~2%.
**Impact:** Direct monetary loss; the cryptographic "proof-of-real-hazard" claim is unmet for the mobile path.
**Fix:** Require hardware-attested (`attestation/index.ts` ECDSA) provenance for reward-bearing hazards, or gate rewards on VLM verification for *all* reward credits (sample only for cost-savings on *non-reward* analytics). Add proof-of-location (cross-witness / RF / speed-plausibility) and per-wallet global rate limits. Stop trusting client `confidence`.

### P0-6 — `/claim-device` binding has no ownership proof (griefing / hijack)
**Evidence:** `claim-device/index.ts:28-88` accepts `{device_id, wallet_pubkey}` with **no signature** from either party, route `auth=NONE`. First caller wins the 1:1 binding.
**Exploit:** Attacker enumerates/guesses a `device_id` and binds it to their own wallet before the legitimate owner (`wallet_taken`/`device_taken` lockout = DoS), or binds the victim's device to an attacker wallet to redirect future association.
**Fix:** Require an Ed25519 proof-of-possession from `wallet_pubkey` and an attestation/secret from the Pi (`device_id`) before binding.

### P0-7 — ECDSA verification disables low-S (signature malleability)
**Evidence:** `attestation/index.ts:121` calls `p256.verify(sigRaw, etHash, pubKey, { prehash: false, lowS: false })`. With `lowS:false`, both `S` and `n−S` verify for the same message.
**Impact:** A captured valid attestation can be malleated into a second distinct-but-valid signature. The monotonic-sequence anti-replay (`:127`) mitigates duplicate *replay*, but any code path that treats the raw signature bytes as a unique identifier (e.g. on-chain `signatureHash`, dedup) is bypassable.
**Fix:** Use `lowS: true` and have the firmware emit canonical low-S signatures (ATECC608A / post-process). Where signatures are used as identifiers, hash the message, not the signature.

### P0-8 — Edge stub-signature fail-open flag (legacy line)
**Evidence:** `src/ecdsa_verify.cpp:88-103` — an all-zero 64-byte signature returns `cfg_.allow_stub_sig`. Wired from env: `src/main.cpp:158` `VIGIA_ALLOW_STUB_SIG` (default `false`, good) → `sensor_bridge.cpp:55-58`.
**Impact:** If the env var is ever set true in the field (or a deploy script defaults it on), all-zero "signatures" are accepted → full edge attestation bypass. This is a fail-open switch in production code.
**Fix:** Delete `allow_stub_sig` from production builds (compile-time `#ifdef VIGIA_DEV` only). Confirm the production ROS2 line (`vigia_ws/...sensor_bridge_node.cpp`) does not carry it. Flag the legacy `src/*.cpp` line as non-shipping if it is.

### P0-9 — Secrets present & not rotated
**Evidence (live):** `vigia-solana-authority`, `vigia/sarvam-api-key`, `vigia/stripe-secret-key`, `vigia/stripe-publishable-key`, `vigia/pgvector` — **all `RotationEnabled=None`**. Roles for slash-node/stripe are correctly scoped to a single secret ARN (verified `get-role-policy`), which is good. No secret value was found logged in source. (Sarvam/Stripe keys are read from env, not hardcoded — confirmed.)
**Fix:** Enable rotation (esp. Stripe + Solana authority — the authority keypair can mint/slash). Move Solana authority to KMS-backed signing if feasible.

---

## 3. Correctness Bugs (P0/P1)

- **P1 — `loadPubkeyHexFile` dead loop** `src/ecdsa_verify.cpp:44-48`: the `for (char& c : hex)` body only `continue`s and mutates nothing — it's a no-op (the actual cleaning happens in the next loop). Harmless but dead/confusing; remove.
- **P1 — Orchestrator score fallback type bug** `orchestrator/index.ts:324-325`: when `agentScore == null`, `discoveryBonus` is set but `totalScore = discoveryBonus! + vlmConfidence*60`; when `agentScore != null`, `discoveryBonus = null` and is later written to traces. Mixed `number|null` arithmetic path is fragile; a non-numeric `verificationScore` regex parse (`:112`) could yield `NaN` → `verdict` comparison `NaN >= 65` is `false` (fails-closed, acceptable) but `Math.round(NaN)` is written to the table. Validate `Number.isFinite(totalScore)`.
- **P1 — Reward credited on fast path before any human/AI verification** (see P0-5). The `tryCreditReward` transaction itself is correct and atomic (`orchestrator/index.ts:151-169`) — the bug is *what triggers it*.
- **P1 — `attestation` upsert writes a non-geohash as `geohash` PK** `attestation/index.ts:210` (`${lat.toFixed(5)},${lon.toFixed(5)}`) while the mobile path writes real ngeohash-7 (`validator/index.ts:88`). Two incompatible PK formats in one table → dedup/queries across paths silently miss each other.
- **P1 — Slash is fire-and-forget, non-blocking, async** `orchestrator/index.ts:280-288`: `lambdaClient.send(...).catch(...)` is not awaited; if the invoke throws synchronously or the Lambda is throttled, the spoofer is never slashed and no record is kept. Combined with 2% sampling this makes slashing largely theatrical.
- **P2 — `verify-hazard-sync` also credits rewards** (`grep` hit) on an interactive/demo path — confirm it shares the dedup lock; an un-deduped demo path is a farming hole.

---

## 4. Reliability & Resilience (P1)

- **DLQ coverage partial:** orchestrator + slash-node have DLQs (wiki + role policy confirms `SlashNodeDLQ`). Stripe/validator/attestation handlers throw to caller (IoT rule has an error-log role `IngestionIoTErrorLog`) — but the **mobile `/telemetry` path has no DLQ**; a 500 just drops the hazard.
- **Idempotency gaps:** Stripe payout (P0-4); `verify-hazard-sync` (P2 above).
- **PITR DISABLED on financial tables:** live check — `RewardsLedgerTable`, `DeviceRegistry`, `HazardsTable` all `PointInTimeRecoveryStatus=DISABLED`. A bad write or deletion to the money ledger is unrecoverable. Enable PITR on all stateful tables (one CDK line: `pointInTimeRecovery: true`).
- **Edge:** firmware COBS decode + monotonic sequence + bounds tests pass (`make test` green; `cobs_roundtrip_test`, `sensor_bridge_test`). Hardware-disconnect/serial-noise handling is tested (rejects bad coords, unknown prefixes). Good.
- **Bedrock agent retry** (`orchestrator/index.ts:71-131`) only retries on substring-matched "timeout"/"Dependency resource" — brittle; use typed error/`$metadata.httpStatusCode` and exponential backoff with jitter.

## 5. Performance & Efficiency (P1/P2)

- **DDB hot-partition risk:** reward dedup key `rwd#<wallet>#<geohash>` (`orchestrator/index.ts:144`) and cooldown `proc#<hazardId>` are well-distributed. But `countVerifiedAtGeohash` (`:36`) does a **Query with FilterExpression on a GSI keyed by status** — every VERIFIED hazard scans under the same `status='VERIFIED'` partition → hot GSI partition + rising RCU cost as the table grows. Add `geohash` to the GSI key, or maintain a counter.
- **Tables are all `PAY_PER_REQUEST`** (verified) — fine for spiky DePIN traffic; watch the VERIFIED-status GSI cost above.
- **Lambda cold start:** TS bundles pull full `@aws-sdk` + `stripe` + `@noble/curves` + `h3-js`; tree-shake/bundle (esbuild) and set memory ≥512MB on the crypto Lambdas. No reserved concurrency on the Sarvam proxy → cost-abuse can exhaust account concurrency.
- **Edge:** ROS2 RT design (SCHED_FIFO core pinning, seqlock /dev/shm ring) is genuinely strong; the documented zero-alloc hot path and IoBinding are the right calls.

## 6. Gaps vs Intended Design

- Wiki `secrets-manager` + README claim Sarvam proxy is JWT-authenticated — **false** (P0-2).
- Wiki `[[fail-closed-vlm]]` "Any VLM failure = QUARANTINE, no reward" is true *only on the 2% path*; the 98% path never invokes VLM and still rewards (design deviation from the stated security model).
- Wiki `[[atomic-reward-credit]]` is correctly implemented — but the upstream trigger is not gated as the design implies.
- Stripe payout flow described as complete; the balance-debit + webhook half is **not implemented** (P0-4).
- `kleidiai-acl` EP marked "planned, not yet built" in wiki — confirmed absent; acceptable as a known gap.
- Two HazardsTable PK formats (P1) indicate the IoT attestation path and mobile path were never reconciled.

## 7. Innovations & Industry-Leading Enhancements

- **Cloud (highest-impact): policy-as-code + zero-trust API.** Add `cdk-nag` (AwsSolutions pack) to CI to *fail the build* on `auth=NONE`, CORS `*`, missing PITR, and unrotated secrets — this single gate would have caught P0-1/2/9 and the PITR gap. Pair with per-route Cognito/IAM authorizers and a signed, single-use, amount-bound payout nonce protocol.
- **Edge (highest-impact): signed OTA model + firmware supply-chain integrity (SLSA + signed artifacts).** The edge runs ONNX models and field firmware with a fail-open stub flag; ship cosign/SLSA-provenance-signed model + firmware bundles verified on-device before load, and remove dev bypass flags from release builds. Tie to ISO 26262 / SOTIF posture for an ADAS.
- **Mobile (highest-impact): hardware-rooted proof-of-location + attestation binding.** Bind reward-bearing telemetry to Play Integrity + the paired Pi's ECDSA attestation and a plausibility model (speed/heading/geo continuity) to defeat Sybil/GPS-spoof farming.
- Cross-cutting: OpenTelemetry tracing edge→IoT→Lambda→chain with SLOs and anomaly alerts on reward-mint rate per wallet; formal reward-economic abuse model; chaos/fault-injection on the VLM-down quarantine path; PII governance for dashcam frames (S3 frames bucket — confirm public-access-block + lifecycle + face/plate blurring for DPDP/GDPR).

## 8. Prioritized Remediation Roadmap

| ID | Finding | Fix | Effort | Dependency |
|---|---|---|---|---|
| P0-4 | Payout no debit / no idempotency | Atomic balance debit + Stripe idempotencyKey + webhook | M | — |
| P0-5 | Reward farming | Gate reward on attestation/VLM; global rate limits; drop client confidence | L | edge attestation |
| P0-3 | Balance↔payout replay | Domain-separate + single-use nonce, amount-bound | S | mobile change |
| P0-2 | Open Sarvam key proxy | Add authorizer + reserved concurrency | S | P0-1 |
| P0-6 | Unsigned claim-device | Require dual proof-of-possession | S | — |
| P0-1 | All routes unauth | Authorizers + WAF + cdk-nag gate | M | — |
| P0-9 | Secrets unrotated | Enable rotation (Stripe, Solana authority) | S | — |
| P0-7 | ECDSA lowS:false | lowS:true + canonical firmware sigs | S | firmware |
| P0-8 | Stub-sig fail-open | Compile out of release builds | S | — |
| P1 | PITR disabled | `pointInTimeRecovery:true` all tables | S | — |
| P1 | HazardsTable PK mismatch | Unify on ngeohash | M | data migration |

## 9. Quick-Win Fixes (low-risk, high-value)

1. **Enable PITR** on all stateful tables (CDK: `pointInTimeRecovery: true`) — zero behavior change, recovers the money ledger. *(verified DISABLED live)*
2. **Stripe idempotency**: `stripe.paymentIntents.create(params, { idempotencyKey: \`payout#${wallet}#${nonce}\` })` at `stripe-payout/index.ts:123` — prevents duplicate intents immediately.
3. **lowS:true** at `attestation/index.ts:121` — one-token change closing malleability (coordinate with firmware to emit canonical sigs first).
4. **Reserved concurrency = small N** on `SarvamProxyFunction` and add the missing JWT check — caps key-abuse blast radius before the full authorizer rollout.
5. **Remove `allow_stub_sig`** path / guard with `#ifdef VIGIA_DEV` in `src/ecdsa_verify.cpp:102` and `src/main.cpp:158`.
6. **Add cdk-nag** to the infra build — surfaces P0-1/2/9 + PITR as build failures going forward.

---
*Read-only AWS calls only; no resource was mutated. Edge `make test` green at audit time.*

---

## 10. Remediation Log (2026-06-20) — implemented & verified

All changes committed (not yet deployed). Builds green: backend+infra `tsc` clean,
`cdk synth` EXIT=0, Android `assembleDemoDebug` BUILD SUCCESSFUL, edge `make test` green.

| ID | Fix shipped | Where |
|----|-------------|-------|
| P0-4 | Atomic balance debit + Stripe idempotency key + rollback; **Stripe webhook** reconciles async settlement (re-credits on failure, idempotent per intent) | `stripe-payout/index.ts`, `stripe-webhook/index.ts`, infra |
| P0-5 | Hardware-attested-only rewards; minted only for `source==='hardware_attested'`, attributed to device-bound wallet; client `confidence`/wallet untrusted; mobile mints nothing (stub) | `orchestrator/index.ts`, `intelligence-stack`, `vigia-stack` |
| P0-3 | Payout proof domain-separated (`VIGIA-PAYOUT`) + single-use via monotonic `last_payout_ts` | `stripe-payout/index.ts`, Android `CopilotViewModel.kt` |
| P0-6 | Dual proof-of-possession on `claim-device` (Ed25519 wallet + ATECC ECDSA device vs PiDeviceRegistry cert); Android sends wallet PoP via `signRaw` | `claim-device/index.ts`, infra, Android pairing |
| P0-8 | Stub-signature acceptance gated behind `VIGIA_DEV`; release builds fail closed | `src/ecdsa_verify.cpp` |
| P1 | PITR on 6 financial/identity tables | ingestion + intelligence stacks |
| P0-2 (partial) | Sarvam proxy `reservedConcurrentExecutions=10` | ingestion-stack |
| Guardrail | **cdk-nag** (AwsSolutions) wired report-only, env-gated (`CDK_NAG=1`); surfaces 226 findings incl. the open `auth=NONE` routes | `bin/vigia.ts` |

## 11. Remaining work (blocked on firmware / ops / coordinated rollout)

| ID | Item | Blocker / next step |
|----|------|---------------------|
| P0-6 (edge) | Pi must ECDSA-sign the `VIGIA-BIND:<device_id>:<wallet>:<ts>` challenge over BLE | **Firmware**: add a "sign binding challenge" GATT op. Until then Android sends empty `device_sig` → server 401 → local-only pairing fallback. |
| P0-1 | API Gateway routes are `auth=NONE` | Add Cognito/Lambda authorizers + WAF — coordinated client rollout (app must send tokens); cdk-nag COG4 already flags every route. |
| P0-2 (full) | Sarvam proxy still unauthenticated (concurrency-capped only) | Cognito JWT verify (JWKS) in Lambda + Android attaches token; or authorizer from P0-1. |
| P0-7 | `attestation` ECDSA uses `lowS:false` (malleability) | **Firmware** must emit canonical low-S sigs first, then flip `lowS:true` (same for `claim-device` device_sig). |
| P0-9 | Secrets unrotated (`stripe-*`, `sarvam`, `solana-authority`, `pgvector`) | **Ops**: enable rotation; third-party keys need custom rotation; move Solana authority to KMS signing. |
| P0-4 (deploy) | Webhook needs `vigia/stripe-webhook-secret` in Secrets Manager + endpoint registered in the Stripe dashboard (`/stripe/webhook`) | **Ops** at deploy time. |
| P0-5 (deploy) | `DeviceBindingsTable` must be populated or hardware rewards won't attribute | **Ops** at deploy time. |
| — | Triage the 226 cdk-nag findings; suppress accepted ones; flip `CDK_NAG=error` as a CI gate | Follow-up once P0-1 authorizers land. |
| P1 | HazardsTable dual PK format (`lat,lon` vs ngeohash) | Unify on ngeohash + data migration. |
| P1 | Slash is fire-and-forget; brittle agent retry; `/telemetry` no DLQ | Reliability hardening. |
