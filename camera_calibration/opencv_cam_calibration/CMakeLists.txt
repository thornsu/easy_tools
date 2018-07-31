cmake_minimum_required(VERSION 2.6)
project(cam_calibration)
SET(CMAKE_BUILD_TYPE Release)
FIND_PACKAGE(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIRS}
  )

add_executable(cam_calibration main.cpp)

target_link_libraries(cam_calibration ${OpenCV_LIBS})

install(TARGETS cam_calibration RUNTIME DESTINATION bin)
