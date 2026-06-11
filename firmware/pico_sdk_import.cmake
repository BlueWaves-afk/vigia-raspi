# Pull in Raspberry Pi Pico SDK (standard import shim).
# Set PICO_SDK_PATH to your pico-sdk checkout, or place pico-sdk next to this repo.

if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    message(STATUS "Using PICO_SDK_PATH from environment ('${PICO_SDK_PATH}')")
endif ()

if (DEFINED ENV{PICO_SDK_FETCH_FROM_GIT} AND (NOT PICO_SDK_FETCH_FROM_GIT))
    set(PICO_SDK_FETCH_FROM_GIT $ENV{PICO_SDK_FETCH_FROM_GIT})
    message(STATUS "Using PICO_SDK_FETCH_FROM_GIT from environment ('${PICO_SDK_FETCH_FROM_GIT}')")
endif ()

if (DEFINED ENV{PICO_SDK_FETCH_FROM_GIT_PATH} AND (NOT PICO_SDK_FETCH_FROM_GIT_PATH))
    set(PICO_SDK_FETCH_FROM_GIT_PATH $ENV{PICO_SDK_FETCH_FROM_GIT_PATH})
    message(STATUS "Using PICO_SDK_FETCH_FROM_GIT_PATH from environment ('${PICO_SDK_FETCH_FROM_GIT_PATH}')")
endif ()

set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Path to Raspberry Pi Pico SDK")
set(PICO_SDK_FETCH_FROM_GIT "${PICO_SDK_FETCH_FROM_GIT}" CACHE BOOL "Fetch Pico SDK from git if missing")
set(PICO_SDK_FETCH_FROM_GIT_PATH "${PICO_SDK_FETCH_FROM_GIT_PATH}" CACHE PATH "Where to fetch Pico SDK")

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake OPTIONAL RESULT_VARIABLE pico_sdk_import_result)

if (NOT pico_sdk_import_result)
    message(FATAL_ERROR
        "Pico SDK not found. Clone it and set PICO_SDK_PATH:\n"
        "  git clone https://github.com/raspberrypi/pico-sdk.git\n"
        "  cd pico-sdk && git submodule update --init\n"
        "  export PICO_SDK_PATH=/path/to/pico-sdk")
endif ()
