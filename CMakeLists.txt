cmake_minimum_required(VERSION 3.0)
project(gaas_perception)

set(CMAKE_CXX_STANDARD 14)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

# for tx2
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
# for x86
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -march=native")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


# thirdparty libs
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Glog REQUIRED)
find_package(Boost COMPONENTS serialization wserialization system REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        std_msgs
        octomap_msgs
        octomap_rviz_plugins
        visualization_msgs
        tf)

set (THIRD_PARTY_LIBS
        ${EIGEN3_LIBS}
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARIES}
        )

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
        ${DBoW3_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${GLOG_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}

)

add_library(perception
        src/PointCloudFilter.cpp
        src/VisionFusion.cpp
        src/VisionFusionNode.cpp
        src/Camera.cpp)

cs_add_executable(VisionFusionNode_run src/VisionFusionNode_run.cpp src/Camera.cpp)
target_link_libraries(VisionFusionNode_run ${perception} ${THIRD_PARTY_LIBS})

##########
# EXPORT #
##########
cs_install()
cs_export()
