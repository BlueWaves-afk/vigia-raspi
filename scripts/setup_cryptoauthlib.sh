#!/usr/bin/env bash
# Clone Microchip cryptoauthlib for Phase 2 live firmware builds.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="${ROOT}/firmware/third_party/cryptoauthlib"

if [[ -d "${DEST}/.git" ]]; then
  echo "cryptoauthlib already present at ${DEST}"
  exit 0
fi

mkdir -p "${ROOT}/firmware/third_party"
git clone --depth 1 https://github.com/MicrochipTech/cryptoauthlib.git "${DEST}"
echo "Cloned cryptoauthlib to ${DEST}"
