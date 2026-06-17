################################################################################
#  VIGIA-RASPI — Multi-Agent Road Intelligence System
#  Target: Raspberry Pi 5 (Cortex-A76 / aarch64 / Debian Trixie)
#
#  Build:
#    mkdir -p build && cd build
#    cmake .. && make -j$(nproc)
################################################################################

cmake_minimum_required(VERSION 3.16)
project(VIGIA-RASPI
    VERSION   1.0.0
    LANGUAGES CXX
)

# ── C++ Standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # clangd / IDE support

# ── Cortex-A76 Optimization Flags ────────────────────────────────────────────
# -mcpu=cortex-a76   : schedule for the Pi 5's quad-core A76 (FEAT_DotProd)
# -O3                : aggressive optimisation (auto-vectorise, inline, unroll)
# -ftree-vectorize   : ensure NEON auto-vectorisation even if -O2 is overridden
# -Wall -Wextra      : catch common mistakes at compile time
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a76 -O3 -ftree-vectorize")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")

# ── Dependency Discovery ─────────────────────────────────────────────────────
include(cmake/Dependencies.cmake)
set(OpenVINO_DIR "/opt/intel/openvino_2025/runtime/cmake")

find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Threads REQUIRED)

# Optional: Add the 3rdparty TBB if needed for parallel processing
set(TBB_DIR "/opt/intel/openvino_2025/runtime/3rdparty/tbb/lib/cmake/TBB")
find_package(TBB QUIET)
# ── Core Library (vigia_lib) ─────────────────────────────────────────────────
# Every agent / engine source file in src/, excluding main.cpp.
set(VIGIA_LIB_SOURCES
    src/perception.cpp
    src/analytical.cpp
    src/coordinator.cpp
    src/fusion.cpp
    src/temporal.cpp
    src/sensor_packet.cpp
    src/sensor_state.cpp
    src/sensor_bridge.cpp
)

add_library(vigia_lib STATIC ${VIGIA_LIB_SOURCES})

target_include_directories(vigia_lib
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include        # project headers (perception.hpp …)
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(vigia_lib
    PUBLIC
        ${OpenCV_LIBS}
        openvino::runtime
        Threads::Threads                   # -pthread (POSIX threads)
        ${CMAKE_DL_LIBS}                   # -ldl     (dlopen / plugin loading)
)

# ── Main Executable (vigia_app) ──────────────────────────────────────────────
add_executable(vigia_app src/main.cpp)

target_link_libraries(vigia_app
    PRIVATE
        vigia_lib
)

# ── Test Executables (one per .cpp in tests/) ────────────────────────────────
file(GLOB TEST_SOURCES "${CMAKE_SOURCE_DIR}/tests/*.cpp")

foreach(test_src ${TEST_SOURCES})
    # tests/system_visual_test.cpp  →  system_visual_test
    get_filename_component(test_name ${test_src} NAME_WE)

    add_executable(${test_name} ${test_src})

    target_link_libraries(${test_name}
        PRIVATE
            vigia_lib
    )
endforeach()

# ── Asset Symlinks (models + test video) ─────────────────────────────────────
# Symlink rather than copy — avoids doubling storage of large .bin / .mp4 files
# on the Pi's SD card.  Works on Linux and macOS.
add_custom_command(
    TARGET vigia_app POST_BUILD
    COMMENT "[vigia] Symlinking assets → ${CMAKE_BINARY_DIR}"
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        "${CMAKE_SOURCE_DIR}/models"
        "${CMAKE_BINARY_DIR}/models"
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        "${CMAKE_SOURCE_DIR}/hazard.mp4"
        "${CMAKE_BINARY_DIR}/hazard.mp4"
)

# ── Build Summary ────────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "╔══════════════════════════════════════════════════════════╗")
message(STATUS "║  VIGIA-RASPI  v${PROJECT_VERSION}                                  ║")
message(STATUS "╠══════════════════════════════════════════════════════════╣")
message(STATUS "║  C++ Standard  : ${CMAKE_CXX_STANDARD}                                     ║")
message(STATUS "║  CXX Flags     : ${CMAKE_CXX_FLAGS}")
message(STATUS "║  OpenCV        : ${OpenCV_VERSION}")
message(STATUS "║  OpenVINO      : found")
message(STATUS "║  Targets       : vigia_app + ${TEST_SOURCES}")
message(STATUS "╚══════════════════════════════════════════════════════════╝")
message(STATUS "")