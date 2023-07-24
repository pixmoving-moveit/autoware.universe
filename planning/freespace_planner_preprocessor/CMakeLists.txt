cmake_minimum_required(VERSION 3.8)
project(freespace_planner_preprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_cmake REQUIRED)

autoware_package()

include_directories(include)

ament_auto_add_library(freespace_planner_preprocessor SHARED
  src/freespace_planner_preprocessor_node.cpp
)
ament_target_dependencies(freespace_planner_preprocessor rclcpp rclcpp_components autoware_auto_planning_msgs autoware_planning_msgs motion_utils tf2 tf2_geometry_msgs tier4_planning_msgs)

rclcpp_components_register_node(freespace_planner_preprocessor
  PLUGIN "freespace_planner_preprocessor::FreeSpacePlannerPreprocessorNode"
  EXECUTABLE freespace_planner_preprocessor_node 
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

ament_auto_package(  
  INSTALL_TO_SHARE
  launch
  config
)