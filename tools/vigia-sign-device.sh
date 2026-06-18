#!/usr/bin/env bash
# vigia-sign-device.sh — generate and sign a device certificate for a VIGIA Pi
#
# This is the SOFTWARE provisioning path, used when:
#   a) The ATECC608A is not yet physically wired (Ben pending), OR
#   b) You are doing local testing with the Pi's software-generated P-256 key.
#
# In production, the ATECC608A generates the keypair internally and the
# Microchip Trust Platform signs the cert — you never touch the private key.
# This script mirrors that flow using OpenSSL for demo/dev.
#
# Usage:
#   bash tools/vigia-sign-device.sh <device_id> [ca_dir] [out_dir]
#
# Example:
#   bash tools/vigia-sign-device.sh vigia-001
#
# Outputs (deploy to Pi via scp):
#   out/<device_id>/device_cert.pem  → /etc/vigia/device_cert.pem
#   out/<device_id>/device_key.pem   → /etc/vigia/device_key.pem  (keep secret)
#   out/<device_id>/pubkey.bin        → /etc/vigia/atecc_pubkey.bin (65-byte raw P-256)
#
# The cert PEM must also be registered in the server DB:
#   psql $DATABASE_URL -c \
#     "INSERT INTO device_registry (device_id, hmac_key, cert_pem)
#      VALUES ('vigia-001', gen_random_bytes(32)::text,
#              '$(cat out/vigia-001/device_cert.pem)');"

set -euo pipefail

DEVICE_ID="${1:?Usage: $0 <device_id> [ca_dir] [out_dir]}"
CA_DIR="${2:-deploy/mosquitto/certs}"
OUT_DIR="${3:-out/${DEVICE_ID}}"

if [[ ! -f "${CA_DIR}/ca.crt" ]] || [[ ! -f "${CA_DIR}/ca.key" ]]; then
    echo "ERROR: CA files not found in ${CA_DIR}. Run tools/vigia-gen-ca.sh first." >&2
    exit 1
fi

mkdir -p "${OUT_DIR}"

echo "[sign-device] Provisioning device: ${DEVICE_ID}"

# ── Generate device P-256 keypair ─────────────────────────────────────────
openssl ecparam -name prime256v1 -genkey -noout -out "${OUT_DIR}/device_key.pem"

# ── Sign with VIGIA CA ────────────────────────────────────────────────────
openssl req -new \
    -key "${OUT_DIR}/device_key.pem" \
    -out "${OUT_DIR}/device.csr" \
    -subj "/C=IN/O=VigiaLabs/CN=${DEVICE_ID}"

openssl x509 -req \
    -in "${OUT_DIR}/device.csr" \
    -CA "${CA_DIR}/ca.crt" \
    -CAkey "${CA_DIR}/ca.key" \
    -CAcreateserial \
    -out "${OUT_DIR}/device_cert.pem" \
    -days 1095 \
    -sha256 \
    -extfile <(printf 'extendedKeyUsage=clientAuth\nsubjectAltName=DNS:%s' "${DEVICE_ID}")

rm "${OUT_DIR}/device.csr"

# ── Export 65-byte raw public key for sensor_bridge_node ECDSA verify ─────
# Format: 0x04 || X[32] || Y[32] (uncompressed P-256 point, IEEE Std 1363)
openssl ec -in "${OUT_DIR}/device_key.pem" -pubout -outform DER 2>/dev/null \
    | tail -c 65 > "${OUT_DIR}/pubkey.bin"

# Verify the file is 65 bytes starting with 0x04
FIRST_BYTE=$(xxd -p -l 1 "${OUT_DIR}/pubkey.bin")
PUBKEY_SIZE=$(wc -c < "${OUT_DIR}/pubkey.bin")
if [[ "${PUBKEY_SIZE}" -ne 65 ]] || [[ "${FIRST_BYTE}" != "04" ]]; then
    echo "ERROR: pubkey.bin is ${PUBKEY_SIZE} bytes with prefix 0x${FIRST_BYTE} (expected 65, 0x04)" >&2
    exit 1
fi

chmod 600 "${OUT_DIR}/device_key.pem"
chmod 644 "${OUT_DIR}/device_cert.pem" "${OUT_DIR}/pubkey.bin"

echo ""
echo "[sign-device] Generated for ${DEVICE_ID}:"
echo "  ${OUT_DIR}/device_cert.pem  → Pi: /etc/vigia/device_cert.pem"
echo "  ${OUT_DIR}/device_key.pem   → Pi: /etc/vigia/device_key.pem  (keep secret)"
echo "  ${OUT_DIR}/pubkey.bin        → Pi: /etc/vigia/atecc_pubkey.bin"
echo ""
echo "[sign-device] Deploy to Pi (replace <pi_ip> with Tailscale IP):"
echo "  PI_IP=100.114.1.98"
echo "  scp ${OUT_DIR}/device_cert.pem vigiasense@\${PI_IP}:/etc/vigia/device_cert.pem"
echo "  scp ${OUT_DIR}/device_key.pem  vigiasense@\${PI_IP}:/etc/vigia/device_key.pem"
echo "  scp ${OUT_DIR}/pubkey.bin       vigiasense@\${PI_IP}:/etc/vigia/atecc_pubkey.bin"
echo "  scp ${CA_DIR}/ca.crt           vigiasense@\${PI_IP}:/etc/vigia/ca_chain.pem"
echo ""
echo "[sign-device] Register device cert in DB:"
CERT_ONELINER=$(awk 'NF{printf "%s\\n", $0}' "${OUT_DIR}/device_cert.pem")
echo "  psql \$DATABASE_URL -c \\"
echo "    \"INSERT INTO device_registry (device_id, hmac_key, cert_pem)"
echo "     VALUES ('${DEVICE_ID}', encode(gen_random_bytes(32),'hex'),"
echo "             '$(cat "${OUT_DIR}/device_cert.pem")');\""
