cmake_minimum_required(VERSION 3.0.2)
project(lipm_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  lipm_msgs
  actionlib
)
find_package(Eigen3)
find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lipm_control
  CATKIN_DEPENDS geometry_msgs roscpp lipm_msgs actionlib  
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
)
add_executable(lipm_control_node src/main.cpp src/control.cpp)
add_dependencies(lipm_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lipm_control_node ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${Boost_LIBRARIES})