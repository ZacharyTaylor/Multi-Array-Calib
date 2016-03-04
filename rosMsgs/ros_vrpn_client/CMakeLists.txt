cmake_minimum_required(VERSION 2.8.0)
project(ros_vrpn_client)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

cs_add_library(vicon_estimator
  src/library/vicon_estimator.cpp
  src/wrapper/vicon_odometry_estimator.cpp
)

cs_add_executable(ros_vrpn_client
	src/ros_vrpn_client.cpp
)

cs_add_executable(vicon_estimation_node
  src/vicon_estimation_node.cpp
)

add_subdirectory(src/test)

target_link_libraries(ros_vrpn_client vicon_estimator ${catkin_LIBRARIES})
target_link_libraries(vicon_estimation_node vicon_estimator ${catkin_LIBRARIES})

cs_install()
cs_export()

