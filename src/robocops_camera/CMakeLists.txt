cmake_minimum_required(VERSION 3.10.2)
project(robocops_camera VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

# set(_opencv_version 4)
# find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui core)  # Ensure core is included if needed
# if(NOT OpenCV_FOUND)
#   set(_opencv_version 3)
#   find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui core)  # Ensure core is included if needed
# endif()

# set(_opencv_version 4.5)
# find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc imgcodecs)  # Specify the exact components and version
# if(NOT OpenCV_FOUND)
#     message(FATAL_ERROR "Required version of OpenCV not found.")
# endif()

find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc imgcodecs)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "Required version of OpenCV not found.")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
find_package(ament_cmake REQUIRED)

find_package(robocops_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
  cv_bridge
  depthai_ros_msgs
  depthai_bridge
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
  robocops_msgs
  std_srvs
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

macro(dai_add_node_ros2 node_name node_src)
  add_executable("${node_name}" "${node_src}")

  ament_target_dependencies("${node_name}"
                          ${dependencies})
  
  target_link_libraries("${node_name}"
                        ${OpenCV_LIBS}
                        depthai::core)
endmacro()

dai_add_node_ros2(robocops_camera_duplos src/duplos.cpp)
dai_add_node_ros2(robocops_camera_imu src/imu.cpp)

install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
FILES_MATCHING PATTERN "*.py")

install(TARGETS
        robocops_camera_duplos
        robocops_camera_imu
        DESTINATION lib/${PROJECT_NAME})

ament_package()