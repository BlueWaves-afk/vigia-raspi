#!/usr/bin/env bash
# build_ort_acl.sh — rebuild ONNX Runtime from source with ARM Compute Library (ACL) EP
#
# Run this ON THE PI. Takes ~2 hours. Run in a tmux session.
# After completion, colcon build will detect onnxruntime_providers_acl.so and
# set VIGIA_HAVE_ACL_EP, dropping inference from ~28ms to ~7ms per frame.
#
# Prerequisites (install once):
#   sudo apt install -y cmake ninja-build scons python3-dev pybind11-dev \
#       libprotobuf-dev protobuf-compiler libflatbuffers-dev git

set -euo pipefail

ORT_VERSION="${ORT_VERSION:-1.20.1}"
ACL_VERSION="${ACL_VERSION:-24.09}"
JOBS="${JOBS:-$(nproc)}"
INSTALL_PREFIX="${INSTALL_PREFIX:-/opt/onnxruntime-kleidiai}"

echo "======================================================================"
echo " VIGIA ORT + ACL EP build"
echo "   ORT:  ${ORT_VERSION}"
echo "   ACL:  ${ACL_VERSION}"
echo "   Jobs: ${JOBS}"
echo "   Out:  ${INSTALL_PREFIX}"
echo "======================================================================"

WORKDIR="$(mktemp -d /tmp/ort-acl-build.XXXX)"
echo "[ort-acl] Build workspace: ${WORKDIR}"
cd "${WORKDIR}"

# ── Step 1: ARM Compute Library ───────────────────────────────────────────
echo "[ort-acl] Cloning ARM Compute Library ${ACL_VERSION}..."
git clone --depth 1 --branch "v${ACL_VERSION}" \
    https://github.com/ARM-software/ComputeLibrary.git acl

cd acl
echo "[ort-acl] Building ACL (scons, -j${JOBS})..."
scons -j"${JOBS}" \
    Werror=0 \
    debug=0 \
    asserts=0 \
    neon=1 \
    opencl=0 \
    os=linux \
    arch=arm64-v8.2-a \
    build=native \
    extra_cxx_flags="-fPIC"

sudo mkdir -p /opt/ComputeLibrary
sudo cp -r arm_compute include /opt/ComputeLibrary/
sudo cp build/*.so* /opt/ComputeLibrary/ 2>/dev/null || true
cd "${WORKDIR}"
echo "[ort-acl] ACL installed to /opt/ComputeLibrary"

# ── Step 2: ONNX Runtime with ACL EP ─────────────────────────────────────
echo "[ort-acl] Cloning ORT v${ORT_VERSION}..."
git clone --depth 1 --branch "v${ORT_VERSION}" \
    https://github.com/microsoft/onnxruntime.git ort

cd ort
echo "[ort-acl] Configuring ORT build (--use_acl)..."
./build.sh \
    --config Release \
    --build_shared_lib \
    --skip_tests \
    --parallel "${JOBS}" \
    --use_acl \
    --acl_home /opt/ComputeLibrary \
    --acl_libs /opt/ComputeLibrary \
    --cmake_extra_defines \
        CMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
        onnxruntime_BUILD_UNIT_TESTS=OFF \
        onnxruntime_USE_FULL_PROTOBUF=ON

echo "[ort-acl] Installing ORT..."
cd build/Linux/Release
sudo make install -j"${JOBS}"
cd "${WORKDIR}"

# ── Step 3: Verify ACL EP .so is present ─────────────────────────────────
ACL_SO="${INSTALL_PREFIX}/lib/libonnxruntime_providers_acl.so"
if [[ ! -f "${ACL_SO}" ]]; then
    echo "ERROR: ACL EP .so not found at ${ACL_SO}" >&2
    echo "       Check build log for ACL linkage errors." >&2
    exit 1
fi

echo ""
echo "[ort-acl] ✅ Build complete!"
echo "   ${ACL_SO}"
echo ""
echo "[ort-acl] Next: rebuild vigia_edge_node — CMakeLists will auto-detect the EP:"
echo "   cd ~/vigia-raspi/vigia_ws"
echo "   colcon build --packages-select vigia_edge_node --cmake-clean-cache"
echo ""
echo "[ort-acl] If cmake still doesn't find it, set:"
echo "   export CMAKE_PREFIX_PATH=${INSTALL_PREFIX}:\${CMAKE_PREFIX_PATH}"

# Cleanup temp dir (keep /opt installs)
rm -rf "${WORKDIR}"
