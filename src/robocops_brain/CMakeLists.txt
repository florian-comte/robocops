cmake_minimum_required(VERSION 3.22)
project(robocops_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robocops_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Install directories
install(DIRECTORY
  behavior_trees launch config
  DESTINATION share/${PROJECT_NAME}
)

#Install C++ nodes
set(AUTONOMY_SOURCES
    src/navigation_behaviors.cpp 
    src/blocking_gpio_behavior.cpp
    src/search_and_grab_behavior.cpp
    src/set_pose_behavior.cpp
    src/can_go_to_zone_behavior.cpp
    src/is_door_open_behavior.cpp
)

set(TARGET_DEPENDS
    ament_index_cpp rclcpp rclcpp_action
    nav2_msgs sensor_msgs cv_bridge image_transport
    behaviortree_cpp tf2 tf2_ros tf2_geometry_msgs yaml-cpp
    control_msgs robocops_msgs 
)

include_directories(include)
add_executable(bt_runner src/bt_runner.cpp ${AUTONOMY_SOURCES})
install(TARGETS
bt_runner
    DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(bt_runner ${TARGET_DEPENDS})
target_link_libraries(bt_runner ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

ament_package()
