#!/usr/bin/env bash
# Provision a VIGIA edge device in device_registry with a random HMAC key.
set -euo pipefail

DEVICE_ID="${1:-vigia-dev-001}"
DB_URL="${DATABASE_URL:-postgresql://vigia:vigia_dev@127.0.0.1:5432/vigia}"

if command -v openssl >/dev/null 2>&1; then
  HMAC_KEY="$(openssl rand -base64 32)"
else
  HMAC_KEY="$(python3 -c 'import secrets; print(secrets.token_urlsafe(32))')"
fi

echo "Provisioning device: ${DEVICE_ID}"

if command -v psql >/dev/null 2>&1; then
  psql "${DB_URL}" -v ON_ERROR_STOP=1 <<SQL
INSERT INTO device_registry (device_id, last_device_seq, hmac_key)
VALUES ('${DEVICE_ID}', 0, '${HMAC_KEY}')
ON CONFLICT (device_id) DO UPDATE SET hmac_key = EXCLUDED.hmac_key;
SQL
else
  python3 - <<PY
import os
import psycopg2

device_id = "${DEVICE_ID}"
hmac_key = "${HMAC_KEY}"
db_url = "${DB_URL}"

conn = psycopg2.connect(db_url)
try:
    with conn.cursor() as cur:
        cur.execute(
            """
            INSERT INTO device_registry (device_id, last_device_seq, hmac_key)
            VALUES (%s, 0, %s)
            ON CONFLICT (device_id) DO UPDATE SET hmac_key = EXCLUDED.hmac_key
            """,
            (device_id, hmac_key),
        )
    conn.commit()
finally:
    conn.close()
PY
fi

echo ""
echo "Device provisioned successfully."
echo "  device_id: ${DEVICE_ID}"
echo "  hmac_key:  ${HMAC_KEY}"
echo ""
echo "Save the key to the edge device (e.g. /etc/vigia/device.key) and set device_id in config/device.yaml."
