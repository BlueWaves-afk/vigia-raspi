cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(vigia_pico_hello C CXX ASM)

set(PICO_BOARD pico2 CACHE STRING "Board type")

pico_sdk_init()

# ── Phase 1 build (bringup — text protocol, no SE) ────────────────────────
add_executable(vigia_pico_hello
    src/main.c
    src/neo_m8n_driver.c
    src/bno085_driver.c
)
target_include_directories(vigia_pico_hello PRIVATE src)
target_compile_definitions(vigia_pico_hello PRIVATE
    VIGIA_IMU_DEBUG_USB=1
    VIGIA_PHASE2=0
)
target_link_libraries(vigia_pico_hello
    pico_stdlib
    hardware_uart
    hardware_gpio
    hardware_spi
    m
)

# ── Phase 2 stub build (COBS + signing pipeline, SE zero-filled) ──────────
# Build with: cmake -DVIGIA_BUILD_PHASE2_STUB=ON ..
option(VIGIA_BUILD_PHASE2_STUB "Build Phase 2 COBS+signing firmware (SE stub)" OFF)

if(VIGIA_BUILD_PHASE2_STUB)
    add_executable(vigia_pico_phase2_stub
        src/main.c
        src/neo_m8n_driver.c
        src/bno085_driver.c
        src/atecc608a_driver.c
        src/cobs_tx_driver.c
    )
    target_include_directories(vigia_pico_phase2_stub PRIVATE src)
    target_compile_definitions(vigia_pico_phase2_stub PRIVATE
        VIGIA_IMU_DEBUG_USB=0
        VIGIA_PHASE2=1
        VIGIA_PHASE2_STUB=1     # zero-fills hash/sig — no I2C
    )
    target_link_libraries(vigia_pico_phase2_stub
        pico_stdlib
        hardware_uart
        hardware_gpio
        hardware_spi
        hardware_i2c
        m
    )
    pico_enable_stdio_usb(vigia_pico_phase2_stub 1)
    pico_enable_stdio_uart(vigia_pico_phase2_stub 0)
    pico_add_extra_outputs(vigia_pico_phase2_stub)
endif()

# ── Phase 2 live build (real ATECC608P — requires SE wired on I2C1) ───────
# Build with: cmake -DVIGIA_BUILD_PHASE2_LIVE=ON -DCRYPTOAUTHLIB_DIR=<path> ..
option(VIGIA_BUILD_PHASE2_LIVE "Build Phase 2 firmware with real ATECC608P signing" OFF)

if(VIGIA_BUILD_PHASE2_LIVE)
    if(NOT DEFINED CRYPTOAUTHLIB_DIR)
        message(FATAL_ERROR "CRYPTOAUTHLIB_DIR must be set for Phase 2 live build")
    endif()

    include(cmake/cryptoauthlib_pico.cmake)

    add_executable(vigia_pico_phase2_live
        src/main.c
        src/neo_m8n_driver.c
        src/bno085_driver.c
        src/atecc608a_driver.c
        src/cobs_tx_driver.c
    )
    target_include_directories(vigia_pico_phase2_live PRIVATE
        src
        ${CRYPTOAUTHLIB_DIR}/lib
    )
    target_compile_definitions(vigia_pico_phase2_live PRIVATE
        VIGIA_IMU_DEBUG_USB=0
        VIGIA_PHASE2=1
        VIGIA_PHASE2_STUB=0
    )
    target_link_libraries(vigia_pico_phase2_live
        pico_stdlib
        hardware_uart
        hardware_gpio
        hardware_spi
        hardware_i2c
        m
    )
    vigia_link_cryptoauthlib(vigia_pico_phase2_live)
    pico_enable_stdio_usb(vigia_pico_phase2_live 1)
    pico_enable_stdio_uart(vigia_pico_phase2_live 0)
    pico_add_extra_outputs(vigia_pico_phase2_live)
endif()
pico_enable_stdio_usb(vigia_pico_hello 1)
pico_enable_stdio_uart(vigia_pico_hello 0)

pico_add_extra_outputs(vigia_pico_hello)
