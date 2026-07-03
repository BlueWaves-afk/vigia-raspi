#!/usr/bin/env bash
# Build one-shot ATECC608 slot-0 provisioning firmware for Pico 2.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FIRMWARE="${ROOT}/firmware"

if [[ -z "${PICO_SDK_PATH:-}" ]]; then
  if [[ -d "${HOME}/pico-sdk" ]]; then
    export PICO_SDK_PATH="${HOME}/pico-sdk"
  else
    echo "Set PICO_SDK_PATH to your pico-sdk checkout." >&2
    exit 1
  fi
fi

if [[ ! -d "${FIRMWARE}/third_party/cryptoauthlib/lib" ]]; then
  "${ROOT}/scripts/setup_cryptoauthlib.sh"
fi

cd "${FIRMWARE}"
cmake -B build-provision -DPICO_BOARD=pico2 \
  -DVIGIA_BUILD_PROVISION=ON \
  -DCRYPTOAUTHLIB_DIR=third_party/cryptoauthlib
cmake --build build-provision -j"$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"

echo ""
echo "UF2: ${FIRMWARE}/build-provision/vigia_pico_provision.uf2"
echo ""
echo "Flash via BOOTSEL, then run:"
echo "  python3 tools/pico_provision_read.py --port /dev/cu.usbmodem* --device-id vigia-pico-001"
