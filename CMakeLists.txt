cmake_minimum_required(VERSION 2.8.3)
project(stair_detection)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge 
    )

find_package(OpenCV REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

# find_package( sophus REQUIRED )
# include_directories( ${sophus_INCLUDE_DIRS} )

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
catkin_package()

add_library(stair_detector_edge SHARED src/stair_detector_geo.cpp
#    src/estimator.cpp
)

target_link_libraries(stair_detector_edge ${catkin_LIBRARIES} ${OpenCV_LIBS} ) # /usr/local/lib/libSophus.so

add_executable(stair_detection_node src/stair_detection_node.cpp )
target_link_libraries(stair_detection_node stair_detector_edge) # /usr/local/lib/libSophus.so

add_executable(test_img src/test_img.cpp)
target_link_libraries(test_img stair_detector_edge)