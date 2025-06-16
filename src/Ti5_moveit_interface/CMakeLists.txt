cmake_minimum_required(VERSION 3.8)
project(Ti5_moveit_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include)

add_library(Ti5_moveit_interface_lib src/Ti5_moveit_interface.cpp) 

ament_target_dependencies(Ti5_moveit_interface_lib
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
)

target_include_directories(Ti5_moveit_interface_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


add_executable(Ti5_node src/Ti5_node.cpp)


target_link_libraries(Ti5_node Ti5_moveit_interface_lib)

ament_target_dependencies(Ti5_node
rclcpp
geometry_msgs
moveit_ros_planning_interface
moveit_core
)


install(DIRECTORY include/
  DESTINATION include/
)


install(TARGETS
  Ti5_moveit_interface_lib
  Ti5_node
  DESTINATION lib/${PROJECT_NAME}
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
