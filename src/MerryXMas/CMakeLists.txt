cmake_minimum_required(VERSION 3.8)
project(merry_xmas)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(freyja_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions( ${EIGEN3_DEFINITIONS} )

include_directories(
  ${Eigen_INCLUDE_DIRS}
)
 

# handler node
add_executable( merry src/merry.cpp )

ament_target_dependencies( merry 
  rclcpp
  std_msgs
  freyja_msgs
)

install(TARGETS
  merry
  DESTINATION lib/${PROJECT_NAME})


ament_package()

