cmake_minimum_required(VERSION 3.0.2)
project(lipm_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  lipm_msgs
  actionlib
  whole_body_ik_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lipm_control
  CATKIN_DEPENDS geometry_msgs roscpp lipm_msgs actionlib  whole_body_ik_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(lipm_control_node src/main.cpp src/control.cpp)
add_dependencies(lipm_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lipm_control_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})