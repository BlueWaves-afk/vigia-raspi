# Integrate Microchip cryptoauthlib for Pico I2C HAL (ATECC608P live signing).
# Expects CRYPTOAUTHLIB_DIR pointing at the cloned repo root.

if(NOT DEFINED CRYPTOAUTHLIB_DIR)
    message(FATAL_ERROR "CRYPTOAUTHLIB_DIR must point to a cryptoauthlib clone")
endif()

if(NOT EXISTS "${CRYPTOAUTHLIB_DIR}/lib/CMakeLists.txt")
    message(FATAL_ERROR "Invalid CRYPTOAUTHLIB_DIR: ${CRYPTOAUTHLIB_DIR}")
endif()

set(ATCA_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(ATCA_NO_POLL OFF CACHE BOOL "" FORCE)
set(ATCA_HAL_I2C ON CACHE BOOL "" FORCE)
set(ATCA_ATECC608_SUPPORT ON CACHE BOOL "" FORCE)
set(ATCA_TNGTLS_SUPPORT OFF CACHE BOOL "" FORCE)
set(ATCA_TNGLORA_SUPPORT OFF CACHE BOOL "" FORCE)
set(ATCA_TFLEX_SUPPORT OFF CACHE BOOL "" FORCE)
set(ATCA_USE_ATCACERT OFF CACHE BOOL "" FORCE)
set(ATCA_PRINTF OFF CACHE BOOL "" FORCE)
set(ATCA_NO_HEAP ON CACHE BOOL "" FORCE)
set(ATCA_ENABLE_DEPRECATED ON CACHE BOOL "" FORCE)

# Host may be macOS/Linux but the target is bare-metal. cryptoauthlib's CMake
# keys off CMAKE host flags (APPLE/UNIX) and would pull hal_linux*.c — exclude
# those after the subdir is added and link our Pico HAL instead.
# Cross-compiling to PICO sets CMAKE_SYSTEM_NAME=PICO with UNIX=1 and APPLE=0,
# so cryptoauthlib's elseif(UNIX) branch pulls hal_linux*.c. Mask that.
set(UNIX OFF)
set(LINUX OFF CACHE BOOL "" FORCE)
set(TWI_SRC "" CACHE INTERNAL "" FORCE)
set(SPI_SRC "" CACHE INTERNAL "" FORCE)

add_subdirectory("${CRYPTOAUTHLIB_DIR}/lib" cryptoauthlib_build EXCLUDE_FROM_ALL)

# Belt-and-suspenders: strip any host HAL that still slipped in.
get_target_property(_cryptoauth_srcs cryptoauth SOURCES)
set(_filtered_srcs "")
foreach(_src IN LISTS _cryptoauth_srcs)
    if(NOT _src MATCHES "hal_linux")
        list(APPEND _filtered_srcs "${_src}")
    endif()
endforeach()
set_property(TARGET cryptoauth PROPERTY SOURCES "${_filtered_srcs}")

# cryptoauthlib's atca_config.h.in uses #cmakedefine (empty value); calib uses #if.
file(READ "${CMAKE_BINARY_DIR}/cryptoauthlib_build/atca_config.h" _atca_cfg)
foreach(_sym IN ITEMS
        ATCA_ATSHA204A_SUPPORT ATCA_ATSHA206A_SUPPORT
        ATCA_ATECC108A_SUPPORT ATCA_ATECC508A_SUPPORT ATCA_ATECC608_SUPPORT
        ATCA_ECC204_SUPPORT ATCA_ECC206_SUPPORT ATCA_TA010_SUPPORT
        ATCA_SHA104_SUPPORT ATCA_SHA105_SUPPORT)
    string(REPLACE "#define ${_sym}\n" "#define ${_sym} 1\n" _atca_cfg "${_atca_cfg}")
endforeach()
file(WRITE "${CMAKE_BINARY_DIR}/cryptoauthlib_build/atca_config.h" "${_atca_cfg}")

# cryptoauthlib links macOS frameworks when APPLE is set; not valid for arm-none-eabi.
if(APPLE)
    set_property(TARGET cryptoauth PROPERTY LINK_LIBRARIES "")
    set_property(TARGET cryptoauth PROPERTY INTERFACE_LINK_LIBRARIES "")
endif()

function(vigia_link_cryptoauthlib target)
    target_sources(${target} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src/atca_hal_pico_i2c.c"
    )
    target_include_directories(${target} PRIVATE
        "${CMAKE_BINARY_DIR}/cryptoauthlib_build"
    )
    target_link_libraries(${target} cryptoauth)
    target_compile_definitions(${target} PRIVATE
        ATCA_NO_HEAP
        ATCA_HAL_I2C
        ATCA_ENABLE_DEPRECATED
    )
endfunction()
