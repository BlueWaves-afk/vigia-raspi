cmake_minimum_required(VERSION 3.10)
project(VIGIA_Road_Intelligence)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Optimization flags for ARM Cortex-A72 / Apple Silicon
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -pthread")

# Allow the caller to override the Kleidi search paths
set(KLEIDIAI_HINT "" CACHE PATH "Root path to the KleidiAI SDK installation")
set(KLEIDICV_HINT "" CACHE PATH "Root path to the KleidiCV SDK installation")

find_package(OpenCV 4.11 REQUIRED COMPONENTS core imgproc dnn)
find_package(OpenVINO REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${OpenVINO_INCLUDE_DIRS}
)

find_library(KLEIDIAI_LIB kleidiai
    HINTS ${KLEIDIAI_HINT} ENV KLEIDIAI_LIB_DIR
    PATH_SUFFIXES lib lib64
)

find_library(KLEIDICV_LIB kleidicv
    HINTS ${KLEIDICV_HINT} ENV KLEIDICV_LIB_DIR
    PATH_SUFFIXES lib lib64
)

if(KLEIDIAI_LIB)
    message(STATUS "Found KleidiAI at ${KLEIDIAI_LIB}")
    add_compile_definitions(VIGIA_USE_KLEIDIAI=1)
else()
    message(WARNING "KleidiAI not found. Set KLEIDIAI_HINT or KLEIDIAI_LIB_DIR to enable optimized kernels.")
endif()

if(KLEIDICV_LIB)
    message(STATUS "Found KleidiCV at ${KLEIDICV_LIB}")
    add_compile_definitions(VIGIA_USE_KLEIDICV=1)
else()
    message(WARNING "KleidiCV not found. Set KLEIDICV_HINT or KLEIDICV_LIB_DIR to enable accelerated preprocessing.")
endif()

add_executable(vigia_main
    src/main.cpp
    src/coordinator.cpp
    src/perception.cpp
    src/analytical.cpp
    src/fusion.cpp
)

set(VIGIA_LINK_LIBS
    ${OpenCV_LIBS}
    openvino::runtime
    pthread
)

if(KLEIDIAI_LIB)
    list(APPEND VIGIA_LINK_LIBS ${KLEIDIAI_LIB})
endif()

if(KLEIDICV_LIB)
    list(APPEND VIGIA_LINK_LIBS ${KLEIDICV_LIB})
endif()

target_link_libraries(vigia_main PRIVATE ${VIGIA_LINK_LIBS})
