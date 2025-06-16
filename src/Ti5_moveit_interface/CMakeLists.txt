cmake_minimum_required(VERSION 3.8)
project(Ti5_moveit_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_pplanning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} src/Ti5_moveit_interface.cpp) 

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
)

add_executable(Ti5_node src/Ti5_node.cpp)
target_link_libraries(Ti5_node ${PROJECT_NAME})
ament_target_dependencies(Ti5_node
  ${PROJECT_NAME}
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
)


install(TARGETS ${PROJECT_NAME} Ti5_node  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
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
