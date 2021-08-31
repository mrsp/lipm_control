cmake_minimum_required(VERSION 3.0.2)
project(lipm_control)
add_compile_options(-std=c++11)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs  
  roscpp
  lipm_msgs
  actionlib
  whole_body_ik_msgs
  whole_body_ik

)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(qpmad)

find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO pinocchio REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lipm_control
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp lipm_msgs actionlib  whole_body_ik_msgs whole_body_ik
  DEPENDS EIGEN3 PINOCCHIO qpmad
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${qpmad_INCLUDE_DIRS}
  )
  link_directories(${PINOCCHIO_LIBRARY_DIRS})

add_library(lipm_control src/main.cpp src/control.cpp src/RobotParameters.cpp src/postureStabilizer.cpp src/ZMPDistributor.cpp src/FootPolygon.cpp src/LeakyIntegrator.cpp)
add_dependencies(lipm_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(lipm_control ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})


add_executable(lipm_control_node src/main.cpp src/control.cpp src/RobotParameters.cpp src/postureStabilizer.cpp src/ZMPDistributor.cpp src/FootPolygon.cpp src/LeakyIntegrator.cpp)
add_dependencies(lipm_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lipm_control_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
