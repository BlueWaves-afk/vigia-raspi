# VIGIA Edge Node — Device Provisioning

One-time setup per Pi. Do this before first deployment.

## 1. Generate Ed25519 Device Key

```bash
# On the Pi (or workstation, then scp)
sudo mkdir -p /etc/vigia
sudo python3 -c "
import nacl.signing, nacl.encoding
k = nacl.signing.SigningKey.generate()
# 32-byte seed (private key)
with open('/etc/vigia/device_ed25519.key', 'wb') as f:
    f.write(bytes(k))
# base58 public key (for device registry)
b58 = k.verify_key.encode(encoder=nacl.encoding.Base58Encoder).decode()
with open('/etc/vigia/device_pubkey.b58', 'w') as f:
    f.write(b58)
print('Public key (base58):', b58)
"
sudo chmod 600 /etc/vigia/device_ed25519.key
sudo chmod 644 /etc/vigia/device_pubkey.b58
```

Install pynacl if not present: `pip3 install pynacl`

## 2. Register Device with AWS Backend

```bash
PUBKEY=$(cat /etc/vigia/device_pubkey.b58)
API_URL="https://sq2ri2n51g.execute-api.us-east-1.amazonaws.com/prod"

curl -s -X POST "${API_URL}/register-device" \
  -H "Content-Type: application/json" \
  -d "{\"device_address\": \"${PUBKEY}\"}" | jq .
```

Expected response:
```json
{"status": "registered", "device_address": "<your_pubkey>"}
```

## 3. Set API URL in ROS2 params

Edit `vigia_ws/src/vigia_edge_node/config/params.yaml`:
```yaml
anti_death_node:
  ros__parameters:
    aws_telemetry_url: "https://sq2ri2n51g.execute-api.us-east-1.amazonaws.com/prod/telemetry"
    device_key_path: "/etc/vigia/device_ed25519.key"
```

## 4. Install system dependencies

```bash
sudo apt install libsodium-dev libcurl4-openssl-dev libgpiod-dev
```

## 5. Verify signing works

```bash
# Quick smoke test — should print a valid base58 signature
python3 -c "
import nacl.signing, nacl.encoding
k = nacl.signing.SigningKey(open('/etc/vigia/device_ed25519.key','rb').read())
msg = b'VIGIA:pothole:12.971600:77.594600:2026-06-17T10:00:00.000Z:0.8200'
sig = k.sign(msg).signature
import base64, hashlib
print('sig len:', len(sig))  # should be 64
print('OK')
"
```

## Signing Contract

The AntiDeathNode signs this exact string (matching `ValidatorFunction/index.ts`):
```
VIGIA:{hazardType}:{lat:.6f}:{lon:.6f}:{timestamp_iso}:{confidence:.4f}
```
Example: `VIGIA:pothole:12.971600:77.594600:2026-06-17T10:00:00.000Z:0.8200`

Signature is Ed25519-detached, base58-encoded — same format as tweetnacl/nacl.sign.detached.

## AWS Pipeline (what happens after POST /telemetry)

```
Pi POST /telemetry
  → ValidatorFunction: Ed25519 verify + device registry check + DynamoDB write (PENDING) + S3 frame
  → HazardsTable DynamoDB Stream INSERT
  → EventBridge Pipe (INSERT filter only)
  → OrchestratorFunction:
      → Nova Lite VLM: "Is this a real road hazard?" (image from S3)
      → Bedrock ReAct Agent: queryHazards + calculateScore
      → verdict: VERIFIED (≥65) or REJECTED
      → if VERIFIED: creditReward (1 VGA pending) + Solana tx (compressed Merkle)
```

The Pi receives `202 PENDING` immediately. Verification is async (Bedrock takes ~5-15s).
