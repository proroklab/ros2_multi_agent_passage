cmake_minimum_required(VERSION 3.5)
project(control_baselines)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(freyja_msgs REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(lib/RVO2)

add_library(librvo2 SHARED IMPORTED)
list(APPEND LIBRVO_INCLUDED_DIRS "${CMAKE_SOURCE_DIR}/lib/RVO2/src")
list(APPEND LIBRVO_INCLUDED_DIRS "${CMAKE_BINARY_DIR}/lib/RVO2/src")
set_target_properties(librvo2 PROPERTIES
        IMPORTED_LOCATION "${CMAKE_BINARY_DIR}/lib/RVO2/src/libRVO.so"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBRVO_INCLUDED_DIRS}"
        )

add_executable(rvo src/rvo.cpp)
ament_target_dependencies(rvo rclcpp std_msgs freyja_msgs)

target_link_libraries(rvo librvo2 Eigen3::Eigen)

install(TARGETS
        rvo
        DESTINATION lib/${PROJECT_NAME})

ament_package()
