cmake_minimum_required(VERSION 3.8)
project(Ti5_arms_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  trajectory_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add Ti5_arms_controller library related compile commands
generate_parameter_library(Ti5_arms_controller_parameters
  src/Ti5_arms_controller.yaml
  include/Ti5_arms_controller/validate_Ti5_arms_controller_parameters.hpp
)
add_library(
  Ti5_arms_controller
  SHARED
  src/Ti5_arms_controller.cpp
)
target_include_directories(Ti5_arms_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(Ti5_arms_controller Ti5_arms_controller_parameters)
ament_target_dependencies(Ti5_arms_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(Ti5_arms_controller PRIVATE "TI5_ARMS_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface Ti5_arms_controller.xml)

install(
  TARGETS Ti5_arms_controller
  EXPORT export_Ti5_arms_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES
  ../../src/bringup/config/Ti5_controller.yaml
  DESTINATION share/${PROJECT_NAME}/config
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  controller_interface
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  Ti5_arms_controller
)



ament_package()
