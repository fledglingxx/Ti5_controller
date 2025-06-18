cmake_minimum_required(VERSION 3.8)
project(Ti5_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


include_directories(
  include
  can_control/include
  ${Eigen3_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/hardware.cpp
  can_control/src/can_hw.cpp
)


target_link_libraries(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/can_control/lib/libcontrolcan.so
)



ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

target_include_directories(${PROJECT_NAME} PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/can_control/include>
 $<INSTALL_INTERFACE:include>
)


#add_executable(Ti5_hardware_interface_node src/hardware.cpp)

pluginlib_export_plugin_description_file(hardware_interface Ti5_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES Ti5_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})



ament_package()
