cmake_minimum_required(VERSION 3.8)
project(robocops_duplos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include headers
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Executable
add_executable(duplos_mapper src/duplos_mapper.cpp src/main.cpp)
ament_target_dependencies(duplos_mapper rclcpp depthai_ros_msgs)

# Install
install(TARGETS duplos_mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
