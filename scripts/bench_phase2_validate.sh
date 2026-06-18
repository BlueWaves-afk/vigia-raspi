#!/usr/bin/env bash
# Bench validation for Phase 2 COBS + signing pipeline.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PORT="/dev/ttyACM0"
DURATION=30
PUBKEY=""

usage() {
  echo "Usage: $0 [--port /dev/ttyACM0] [--duration SEC] [--pubkey PATH]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --pubkey) PUBKEY="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1"; usage; exit 1 ;;
  esac
done

echo "== Phase 2 bench validation =="
echo "Port: ${PORT}, duration: ${DURATION}s"

CMD=(python3 "${ROOT}/tools/pico_packet_monitor.py" --port "${PORT}" --duration "${DURATION}")
if [[ -n "${PUBKEY}" ]]; then
  CMD+=(--pubkey "${PUBKEY}")
fi

"${CMD[@]}"

echo ""
echo "Optional: run C++ COBS round-trip test"
if command -v clang++ >/dev/null 2>&1; then
  clang++ -std=c++17 "${ROOT}/tests/cobs_roundtrip_test.cpp" \
    "${ROOT}/src/cobs.cpp" \
    "${ROOT}/src/signed_et_packet.cpp" \
    -I"${ROOT}/include" -O2 -o /tmp/cobs_roundtrip_test
  /tmp/cobs_roundtrip_test
else
  echo "clang++ not found — skip round-trip test"
fi

echo "Done."
