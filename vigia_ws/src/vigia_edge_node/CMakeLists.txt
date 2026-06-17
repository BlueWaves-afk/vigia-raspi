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

# ONNX Runtime — installed to /usr/local or /opt/onnxruntime
find_path(ORT_INCLUDE_DIR onnxruntime/onnxruntime_cxx_api.h
    PATHS /usr/local/include /opt/onnxruntime/include)
find_library(ORT_LIB onnxruntime
    PATHS /usr/local/lib /opt/onnxruntime/lib)

if(NOT ORT_INCLUDE_DIR OR NOT ORT_LIB)
    message(WARNING "ONNX Runtime not found — VisionNode and DepthNode will not build")
    set(HAVE_ORT FALSE)
else()
    set(HAVE_ORT TRUE)
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
        $<$<BOOL:${HAVE_ORT}>:${ORT_INCLUDE_DIR}>
)

ament_target_dependencies(vigia_edge_node
    rclcpp sensor_msgs std_msgs vigia_msgs)

target_link_libraries(vigia_edge_node
    ${OpenCV_LIBS}
    Eigen3::Eigen
    pthread
    rt
    $<$<BOOL:${HAVE_ORT}>:${ORT_LIB}>
)

install(TARGETS vigia_edge_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config)

ament_package()
