#!/usr/bin/env bash
# vigia-gen-ca.sh — generate VIGIA Root CA + Mosquitto server cert
#
# Run ONCE on the ingest server. Outputs go into deploy/mosquitto/certs/.
# In production these are replaced by Microchip Trust Platform CA certs.
#
# Usage:  bash tools/vigia-gen-ca.sh [output_dir]
# Default output_dir: deploy/mosquitto/certs

set -euo pipefail

OUT="${1:-deploy/mosquitto/certs}"
mkdir -p "${OUT}"

echo "[vigia-gen-ca] Generating VIGIA Root CA (P-256, SHA-256) → ${OUT}/ca.*"

# ── Root CA ────────────────────────────────────────────────────────────────
openssl ecparam -name prime256v1 -genkey -noout -out "${OUT}/ca.key"
openssl req -new -x509 \
    -key "${OUT}/ca.key" \
    -out "${OUT}/ca.crt" \
    -days 3650 \
    -subj "/C=IN/O=VigiaLabs/CN=VIGIA-Root-CA" \
    -extensions v3_ca \
    -addext "basicConstraints=critical,CA:TRUE" \
    -addext "keyUsage=critical,keyCertSign,cRLSign" \
    -sha256

echo "[vigia-gen-ca] Root CA generated: ${OUT}/ca.crt"

# ── Mosquitto server cert ──────────────────────────────────────────────────
SERVER_CN="${VIGIA_SERVER_CN:-vigia-broker}"

openssl ecparam -name prime256v1 -genkey -noout -out "${OUT}/server.key"
openssl req -new \
    -key "${OUT}/server.key" \
    -out "${OUT}/server.csr" \
    -subj "/C=IN/O=VigiaLabs/CN=${SERVER_CN}"

openssl x509 -req \
    -in "${OUT}/server.csr" \
    -CA "${OUT}/ca.crt" \
    -CAkey "${OUT}/ca.key" \
    -CAcreateserial \
    -out "${OUT}/server.crt" \
    -days 825 \
    -sha256 \
    -extfile <(printf "subjectAltName=DNS:%s,DNS:localhost,IP:127.0.0.1" "${SERVER_CN}")

rm "${OUT}/server.csr"

echo "[vigia-gen-ca] Server cert generated: ${OUT}/server.crt (CN=${SERVER_CN})"

# ── Permissions ────────────────────────────────────────────────────────────
chmod 600 "${OUT}/ca.key" "${OUT}/server.key"
chmod 644 "${OUT}/ca.crt" "${OUT}/server.crt"

echo ""
echo "[vigia-gen-ca] Done. Next step: register devices with tools/vigia-sign-device.sh"
echo "  Upload ca.crt to each device at /etc/vigia/ca_chain.pem"
