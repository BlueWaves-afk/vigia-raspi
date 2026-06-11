cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(vigia_pico_hello C CXX ASM)

set(PICO_BOARD pico2 CACHE STRING "Board type")

pico_sdk_init()

add_executable(vigia_pico_hello
    src/main.c
    src/neo_m8n_driver.c
)

target_include_directories(vigia_pico_hello PRIVATE src)

target_link_libraries(vigia_pico_hello
    pico_stdlib
    hardware_uart
    hardware_gpio
)

pico_enable_stdio_usb(vigia_pico_hello 1)
pico_enable_stdio_uart(vigia_pico_hello 0)

pico_add_extra_outputs(vigia_pico_hello)
