cmake_minimum_required(VERSION 3.16)
project(vigia_edge_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Cortex-A76 optimisation — Pi 5 specific.
add_compile_options(-mcpu=cortex-a76 -O3 -ftree-vectorize -Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(vigia_msgs         REQUIRED)
find_package(OpenCV             REQUIRED COMPONENTS core imgproc videoio)
find_package(Eigen3             REQUIRED)

# libsodium — Ed25519 signing for AWS telemetry (AntiDeathNode)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SODIUM REQUIRED libsodium)

# libcurl — HTTPS POST to API Gateway (AntiDeathNode)
find_package(CURL REQUIRED)

# libgpiod — GPIO edge-wait for UPS POWER_FAIL (AntiDeathNode)
pkg_check_modules(GPIOD libgpiod)
if(GPIOD_FOUND)
    message(STATUS "libgpiod found — real GPIO edge-wait enabled")
else()
    message(STATUS "libgpiod not found — GPIO poll fallback will be used")
endif()

# mbedTLS — for Phase 2 ECDSA signature verification (sig_valid flag)
# Optional: if not found, SensorBridgeNode compiles but sig_valid stays false
find_package(MbedTLS QUIET)
if(MbedTLS_FOUND)
    set(HAVE_MBEDTLS TRUE)
    message(STATUS "mbedTLS found — Phase 2 ECDSA verify enabled")
else()
    set(HAVE_MBEDTLS FALSE)
    message(STATUS "mbedTLS not found — Phase 2 sig_valid will always be false")
endif()

# ONNX Runtime — headers from KleidiAI build, lib from prebuilt 1.20.1
# KleidiAI dir has include/onnxruntime/ structure matching our #include paths.
# Prebuilt aarch64 1.20.1 supplies libonnxruntime.so (KleidiAI build lost the lib).
find_path(ORT_INCLUDE_DIR onnxruntime/onnxruntime_cxx_api.h
    PATHS /opt/onnxruntime-kleidiai/include /usr/local/include)
find_library(ORT_LIB onnxruntime
    PATHS /opt/onnxruntime-linux-aarch64-1.20.1/lib /usr/local/lib)

if(NOT ORT_INCLUDE_DIR OR NOT ORT_LIB)
    message(WARNING "ONNX Runtime not found — VisionNode and DepthNode will not build")
    set(HAVE_ORT FALSE)
else()
    set(HAVE_ORT TRUE)
endif()

# KleidiAI ACL Execution Provider — only present when ORT is rebuilt from source
# with --use_acl + ARM Compute Library. The prebuilt 1.20.1 is CPU-EP only.
# KleidiAI INT8 UDOT MLAS kernels are baked into the ORT lib at build time (not an
# explicit EP call) — so this only guards the ACL provider .so linkage.
find_library(ACL_EP_LIB onnxruntime_providers_acl
    PATHS /opt/onnxruntime-kleidiai/lib /opt/onnxruntime-linux-aarch64-1.20.1/lib /usr/local/lib)
if(ACL_EP_LIB)
    message(STATUS "KleidiAI ACL EP found: ${ACL_EP_LIB} — ACL provider enabled")
    set(HAVE_ACL_EP TRUE)
else()
    message(STATUS "KleidiAI ACL EP not found — CPU EP only (rebuild ORT with --use_acl to enable)")
    set(HAVE_ACL_EP FALSE)
endif()

# ── vigia_edge_node executable ─────────────────────────────────────────────

set(NODE_SRCS
    src/main.cpp
    src/camera_node.cpp
    src/fusion_node.cpp
    src/sensor_bridge_node.cpp
    src/anti_death_node.cpp
)

if(HAVE_ORT)
    list(APPEND NODE_SRCS src/vision_node.cpp src/depth_node.cpp)
endif()

add_executable(vigia_edge_node ${NODE_SRCS})

target_include_directories(vigia_edge_node
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIRS}
        ${SODIUM_INCLUDE_DIRS}
        ${CURL_INCLUDE_DIRS}
        $<$<BOOL:${HAVE_ORT}>:${ORT_INCLUDE_DIR}>
        $<$<BOOL:${GPIOD_FOUND}>:${GPIOD_INCLUDE_DIRS}>
)

target_compile_definitions(vigia_edge_node PRIVATE
    $<$<BOOL:${HAVE_MBEDTLS}>:VIGIA_HAVE_MBEDTLS>
    $<$<BOOL:${GPIOD_FOUND}>:VIGIA_HAVE_LIBGPIOD>
    $<$<BOOL:${HAVE_ACL_EP}>:VIGIA_HAVE_ACL_EP>
)

ament_target_dependencies(vigia_edge_node
    rclcpp sensor_msgs std_msgs vigia_msgs)

target_link_libraries(vigia_edge_node
    ${OpenCV_LIBS}
    Eigen3::Eigen
    pthread
    rt
    ${SODIUM_LIBRARIES}
    ${CURL_LIBRARIES}
    $<$<BOOL:${HAVE_ORT}>:${ORT_LIB}>
    $<$<BOOL:${HAVE_MBEDTLS}>:MbedTLS::mbedcrypto>
    $<$<BOOL:${GPIOD_FOUND}>:${GPIOD_LIBRARIES}>
    $<$<BOOL:${HAVE_ACL_EP}>:${ACL_EP_LIB}>
)

install(TARGETS vigia_edge_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config)

install(FILES systemd/vigia-edge.service
    DESTINATION share/${PROJECT_NAME}/systemd)

ament_package()
