cmake_minimum_required(VERSION 3.16)
project(robocops_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  robocops_description
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

## COMPILE
add_library(
  robocops_control
  SHARED
  hardware/diffbot_system.cpp
  hardware/utils.cpp
  hardware/arduino_comms.cpp
)

target_compile_features(robocops_control PUBLIC cxx_std_17)

target_include_directories(robocops_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/robocops_control>
  ${LIBSERIAL_INCLUDE_DIRS} 
)

target_link_libraries(robocops_control
  PUBLIC ${LIBSERIAL_LIBRARIES}
)

ament_target_dependencies(
  robocops_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface robocops_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/robocops_control
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/robocops_control
)


install(TARGETS robocops_control
  EXPORT export_robocops_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_robocops_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
