cmake_minimum_required(VERSION 3.16)
project(vigia_edge_node)

# Phase 2 bench: build only sensor_bridge_node (skip monolith + AntiDeath/BLE/MQTT).
option(VIGIA_BRIDGE_ONLY "Build sensor_bridge_node only" OFF)

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
# AntiDeathNode uses the v2 API (gpiod_chip_request_lines / gpiod_edge_event_buffer).
# Both v1 and v2 are accepted; v1 will also compile but the runtime path uses v2 structs
# only present from 2.0 onward, so we gate on version >= 2.0.
pkg_check_modules(GPIOD libgpiod)
if(GPIOD_FOUND)
    if(GPIOD_VERSION VERSION_GREATER_EQUAL "2.0")
        message(STATUS "libgpiod ${GPIOD_VERSION} (v2 API) — GPIO edge-wait enabled")
    else()
        message(STATUS "libgpiod ${GPIOD_VERSION} (v1) — v2 API required; GPIO poll fallback will be used")
        set(GPIOD_FOUND FALSE)
    endif()
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

# Paho MQTT C++ — Phase 5 MQTT uplink for AntiDeathNode (doc 05 §7)
# Install on Pi: sudo apt install libpaho-mqtt-dev libpaho-mqttpp-dev
# Debian Trixie ships no .pc or cmake config — use find_library + find_path directly.
find_library(PAHO_MQTTPP_LIB paho-mqttpp3
    PATHS /usr/lib/aarch64-linux-gnu /usr/local/lib)
find_library(PAHO_MQTT3AS_LIB paho-mqtt3as
    PATHS /usr/lib/aarch64-linux-gnu /usr/local/lib)
find_path(PAHO_MQTTPP_INCLUDE_DIR mqtt/async_client.h
    PATHS /usr/include /usr/local/include)
if(PAHO_MQTTPP_LIB AND PAHO_MQTTPP_INCLUDE_DIR)
    message(STATUS "Paho MQTT C++ found: ${PAHO_MQTTPP_LIB} — Phase 5 MQTT path enabled")
    set(HAVE_PAHO_MQTT TRUE)
    set(PAHO_MQTTPP_LIBRARIES ${PAHO_MQTTPP_LIB} ${PAHO_MQTT3AS_LIB})
    set(PAHO_MQTTPP_INCLUDE_DIRS ${PAHO_MQTTPP_INCLUDE_DIR})
else()
    message(STATUS "Paho MQTT C++ not found — AntiDeathNode uses legacy HTTPS fallback")
    set(HAVE_PAHO_MQTT FALSE)
endif()

# msgpack-cxx — Phase 5 MsgPack serialization (header-only C++ API)
# Install on Pi: sudo apt install libmsgpack-cxx-dev
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp
    PATHS /usr/include /usr/local/include)
if(MSGPACK_INCLUDE_DIR)
    message(STATUS "msgpack-cxx found at ${MSGPACK_INCLUDE_DIR}")
    set(HAVE_MSGPACK TRUE)
else()
    message(STATUS "msgpack-cxx not found — MsgPack serialization disabled")
    set(HAVE_MSGPACK FALSE)
endif()

# sdbus-c++ — BlueZ D-Bus GATT server for BleGattNode (§3, app_dashcam_integration.md)
pkg_check_modules(SDBUS sdbus-c++)
if(SDBUS_FOUND)
    message(STATUS "sdbus-c++ ${SDBUS_VERSION} found — BleGattNode will build")
    set(HAVE_SDBUS TRUE)
else()
    message(STATUS "sdbus-c++ not found — BleGattNode will not build (install libsdbus-c++-dev)")
    set(HAVE_SDBUS FALSE)
endif()

# ── vigia_edge_node executable (full stack — skipped when VIGIA_BRIDGE_ONLY) ─

if(NOT VIGIA_BRIDGE_ONLY)
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

    if(HAVE_SDBUS)
        list(APPEND NODE_SRCS src/ble_gatt_node.cpp)
    endif()

    if(HAVE_PAHO_MQTT AND HAVE_MSGPACK)
        list(APPEND NODE_SRCS src/hazard_uplink_node.cpp)
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
            $<$<BOOL:${HAVE_SDBUS}>:${SDBUS_INCLUDE_DIRS}>
            $<$<BOOL:${HAVE_PAHO_MQTT}>:${PAHO_MQTTPP_INCLUDE_DIRS}>
            $<$<BOOL:${HAVE_MSGPACK}>:${MSGPACK_INCLUDE_DIR}>
    )

    target_compile_definitions(vigia_edge_node PRIVATE
        $<$<BOOL:${HAVE_ORT}>:VIGIA_HAVE_ORT>
        $<$<BOOL:${HAVE_MBEDTLS}>:VIGIA_HAVE_MBEDTLS>
        $<$<BOOL:${GPIOD_FOUND}>:VIGIA_HAVE_LIBGPIOD>
        $<$<BOOL:${HAVE_ACL_EP}>:VIGIA_HAVE_ACL_EP>
        $<$<BOOL:${HAVE_SDBUS}>:VIGIA_HAVE_SDBUS>
        # Phase 5: both Paho and msgpack must be found for the MQTT+MsgPack path.
        $<$<AND:$<BOOL:${HAVE_PAHO_MQTT}>,$<BOOL:${HAVE_MSGPACK}>>:VIGIA_HAVE_PAHO_MQTT>
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
        $<$<BOOL:${HAVE_SDBUS}>:${SDBUS_LIBRARIES}>
        $<$<BOOL:${HAVE_PAHO_MQTT}>:${PAHO_MQTTPP_LIBRARIES}>
    )

    install(TARGETS vigia_edge_node
        DESTINATION lib/${PROJECT_NAME})
else()
    message(STATUS "VIGIA_BRIDGE_ONLY=ON — skipping vigia_edge_node monolith")
endif()

# Standalone bridge — build/run without AntiDeath/BLE/MQTT (Phase 2 bench).
if(HAVE_MBEDTLS)
    add_executable(sensor_bridge_node
        src/sensor_bridge_main.cpp
        src/sensor_bridge_node.cpp
    )
    target_include_directories(sensor_bridge_node
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_compile_definitions(sensor_bridge_node PRIVATE VIGIA_HAVE_MBEDTLS)
    ament_target_dependencies(sensor_bridge_node rclcpp vigia_msgs)
    target_link_libraries(sensor_bridge_node
        pthread
        MbedTLS::mbedcrypto
    )
    install(TARGETS sensor_bridge_node DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config)

install(FILES systemd/vigia-edge.service
    DESTINATION share/${PROJECT_NAME}/systemd)

# ── Unit tests — pure C++, no ROS2/ORT/BlueZ deps (run with `colcon test`) ──
if(BUILD_TESTING)
    foreach(_t test_ble_frame_codec test_vigia_rri)
        add_executable(${_t} test/${_t}.cpp)
        target_include_directories(${_t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
        add_test(NAME ${_t} COMMAND ${_t})
    endforeach()
endif()

ament_package()
