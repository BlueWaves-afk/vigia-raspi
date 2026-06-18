#!/usr/bin/env bash
# Build Phase 2 live firmware (ATECC608P + cryptoauthlib).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FIRMWARE="${ROOT}/firmware"

if [[ -z "${PICO_SDK_PATH:-}" ]]; then
  if [[ -d "${HOME}/pico-sdk" ]]; then
    export PICO_SDK_PATH="${HOME}/pico-sdk"
  else
    echo "Set PICO_SDK_PATH to your pico-sdk checkout." >&2
    echo "  export PICO_SDK_PATH=\$HOME/pico-sdk" >&2
    exit 1
  fi
fi

if [[ ! -d "${FIRMWARE}/third_party/cryptoauthlib/lib" ]]; then
  "${ROOT}/scripts/setup_cryptoauthlib.sh"
fi

cd "${FIRMWARE}"
cmake -B build-phase2-live -DPICO_BOARD=pico2 \
  -DVIGIA_BUILD_PHASE2_LIVE=ON \
  -DCRYPTOAUTHLIB_DIR=third_party/cryptoauthlib
cmake --build build-phase2-live -j"$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"

echo ""
echo "UF2: ${FIRMWARE}/build-phase2-live/vigia_pico_phase2_live.uf2"
