cmake_minimum_required(VERSION 2.8.3)
project(vins_7)

# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/lib")
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

set(Pangolin_DIR "/usr/local/lib")
set(used_pangolin_so
    ${Pangolin_DIR}/libpango_windowing.so
    ${Pangolin_DIR}/libpango_windowing.so.0
    ${Pangolin_DIR}/libpango_windowing.so.0.9.3
)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

add_library(camera_model SHARED
    src/camera_models/chessboard/Chessboard.cc
    src/camera_models/calib/CameraCalibration.cc
    src/camera_models/camera_models/Camera.cc
    src/camera_models/camera_models/CameraFactory.cc
    src/camera_models/camera_models/CostFunctionFactory.cc
    src/camera_models/camera_models/PinholeCamera.cc
    src/camera_models/camera_models/CataCamera.cc
    src/camera_models/camera_models/EquidistantCamera.cc
    src/camera_models/camera_models/ScaramuzzaCamera.cc
    src/camera_models/sparse_graph/Transform.cc
    src/camera_models/gpl/gpl.cc
    src/camera_models/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
message(WARNING "cameral build done")

ADD_LIBRARY(MyVio SHARED
    src/System.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/feature_tracker.cpp

    src/utility/utility.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp

    src/backend/vertex.cc
    src/backend/edge.cc
    src/backend/problem.cc
    src/backend/vertex_pose.cc
    src/backend/edge_reprojection.cc
    src/backend/edge_imu.cc
    src/backend/edge_prior.cpp
    src/backend/loss_function.cc
    src/backend/imu_integration.cc
    )

target_link_libraries(MyVio  
    ${OpenCV_LIBS}  
    ${CERES_LIBRARIES}
    ${Pangolin_LIBRARIES}
    camera_model
    ${used_pangolin_so}
)

message(WARNING "MyVio build done")

add_executable(run_euroc test/run_euroc.cpp)
target_link_libraries(run_euroc 
    MyVio  
    -lpthread
    ${used_pangolin_so}
) 

add_executable(testCurveFitting test/CurveFitting.cpp)
target_link_libraries(testCurveFitting 
MyVio
${used_pangolin_so}
)

