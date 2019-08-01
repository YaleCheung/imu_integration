cmake_minimum_required(VERSION 2.8.3)
project(IMUIntegration)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -fsanitize=address")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -fsanitize=address")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

include_directories(
    include
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR})

catkin_package(
            CATKIN_DEPENDS roscpp std_msgs sensor_msgs
            DEPENDS EIGEN3 
            INCLUDE_DIRS include)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(imuIntegration src/imu_integration.cc)
target_link_libraries(imuIntegration stdc++fs ${catkin_LIBRARIES})
