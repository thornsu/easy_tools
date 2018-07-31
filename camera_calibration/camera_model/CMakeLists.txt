cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator_no_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread -fopenmp")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(Pangolin REQUIRED)

message("CERES_INCLUDE_DIR:" ${CERES_INCLUDE_DIR}  " CERES_LIBS: "  ${CERES_LIBRARIES} )
message("EIGEN3_INCLUDE_DIR:" ${EIGEN3_INCLUDE_DIR}  " EIGEN3_LIBS: "  ${Eigen3_LIBRARIES} )
message("OPENCV_INCLUDE_DIR:" ${OpenCV_INCLUDE_DIRS}  " OPENCV_LIBS: "  ${OpenCV_LIBS} )
message("BOOST_INCLUDE_DIR:" ${Boost_INCLUDE_DIRS}  " BOOST_LIBS: "  ${Boost_LIBRARIES} )
message("Pangolin_INCLUDE_DIR:" ${Pangolin_INCLUDE_DIRS}  " Pangolin_LIBS: "  ${Pangolin_LIBRARIES} )


include_directories(
		${PROJECT_SOURCE_DIR}/include/camera_models
		${PROJECT_SOURCE_DIR}/include/
 		${OpenCV_INCLUDE_DIRS}
		${Pangolin_INCLUDE_DIRS}
		${Boost_INCLUDE_DIRS}
		${CERES_INCLUDE_DIRS}
		${Eigen_INCLUDE_DIRS}
)

add_executable(camera_calib
    ${PROJECT_SOURCE_DIR}/src/chessboard/Chessboard.cc
    ${PROJECT_SOURCE_DIR}/src/calib/CameraCalibration.cc
    ${PROJECT_SOURCE_DIR}/src/camera_models/Camera.cc
    ${PROJECT_SOURCE_DIR}/src/camera_models/CameraFactory.cc
    ${PROJECT_SOURCE_DIR}/src/camera_models/CostFunctionFactory.cc
    ${PROJECT_SOURCE_DIR}/src/camera_models/PinholeCamera.cc
    ${PROJECT_SOURCE_DIR}/src/camera_models/CataCamera.cc
    ${PROJECT_SOURCE_DIR}/src/camera_models/EquidistantCamera.cc
    ${PROJECT_SOURCE_DIR}/src/camera_models/ScaramuzzaCamera.cc
    ${PROJECT_SOURCE_DIR}/src/sparse_graph/Transform.cc
    ${PROJECT_SOURCE_DIR}/src/gpl/gpl.cc
    ${PROJECT_SOURCE_DIR}/src/gpl/EigenQuaternionParameterization.cc
    ${PROJECT_SOURCE_DIR}/src/intrinsic_calib.cc
)

target_link_libraries(camera_calib ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})




