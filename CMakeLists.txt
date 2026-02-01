cmake_minimum_required(VERSION 3.16)
project(vigia LANGUAGES CXX)

# --------------------
# Global settings
# --------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --------------------
# Output directories
# --------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# --------------------
# Warnings (safe)
# --------------------
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# --------------------
# Dependencies
# --------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Dependencies)

# --------------------
# Include paths
# --------------------
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# --------------------
# Core library
# --------------------
add_library(vigia_core
    src/analytical.cpp
    src/temporal.cpp
    src/fusion.cpp
    src/coordinator.cpp
    src/perception.cpp
)

target_link_libraries(vigia_core
    OpenCV_LIBS
    openvino::runtime
    pthread
)

# --------------------
# Tests
# --------------------
enable_testing()

add_executable(analytical_test tests/analytical_test.cpp)
target_link_libraries(analytical_test vigia_core)

add_executable(temporal_test tests/temporal_test.cpp)
target_link_libraries(temporal_test vigia_core)

add_executable(fusion_test tests/fusion_test.cpp)
target_link_libraries(fusion_test vigia_core)

add_executable(coordinator_test tests/coordinator_test.cpp)
target_link_libraries(coordinator_test vigia_core)

add_test(NAME analytical_test COMMAND analytical_test)
add_test(NAME temporal_test COMMAND temporal_test)
add_test(NAME fusion_test COMMAND fusion_test)
add_test(NAME coordinator_test COMMAND coordinator_test)
