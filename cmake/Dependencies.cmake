################################################################################
#  Dependencies.cmake — External library discovery for VIGIA-RASPI
#
#  Hard requirements:
#    • OpenCV  ≥ 4.x   (built with KleidiCV HAL + TBB backend)
#    • OpenVINO 2025    (CPU plugin with KleidiAI INT8 on aarch64 / Cortex-A76)
################################################################################

# ── POSIX Threads ─────────────────────────────────────────────────────────────
# CMake's FindThreads sets Threads::Threads, which adds -pthread portably.
find_package(Threads REQUIRED)

# ── OpenCV ────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED
    COMPONENTS core imgproc imgcodecs videoio highgui
)

if(NOT OpenCV_FOUND)
    message(FATAL_ERROR
        "[vigia] OpenCV not found.\n"
        "        Ensure the KleidiCV+TBB build is installed and "
        "CMAKE_PREFIX_PATH includes its location."
    )
endif()

message(STATUS "[vigia] OpenCV ${OpenCV_VERSION}  (${OpenCV_INSTALL_PATH})")

# Sanity-check: confirm TBB parallel backend is available
if(OpenCV_LIBS MATCHES "tbb" OR EXISTS "${OpenCV_INSTALL_PATH}/lib/libopencv_core.so")
    message(STATUS "[vigia]   TBB backend: expected (linked at build time)")
endif()

# ── OpenVINO ──────────────────────────────────────────────────────────────────
# Requires `source /opt/intel/openvino/setupvars.sh` (or equivalent) before cmake.
find_package(OpenVINO REQUIRED
    COMPONENTS Runtime
)

if(NOT OpenVINO_FOUND)
    message(FATAL_ERROR
        "[vigia] OpenVINO not found.\n"
        "        Source setupvars.sh or set OpenVINO_DIR / CMAKE_PREFIX_PATH."
    )
endif()

message(STATUS "[vigia] OpenVINO Runtime: found")
