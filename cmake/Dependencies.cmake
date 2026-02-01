# --------------------
# OpenCV
# --------------------
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# --------------------
# OpenVINO
# --------------------
find_package(OpenVINO REQUIRED)
message(STATUS "Found OpenVINO")
