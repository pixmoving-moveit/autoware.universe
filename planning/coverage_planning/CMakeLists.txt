cmake_minimum_required(VERSION 3.14)
project(coverage_planning)

add_compile_options(-std=c++14)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(coverage_planning_core SHARED
  src/coverage_planning_core.cpp
)

ament_auto_add_library(coverage_planning SHARED
  src/coverage_planning.cpp
)

target_link_libraries(coverage_planning
  coverage_planning_core
)

ament_auto_add_executable(coverage_planning_node
  src/coverage_planning_node.cpp
)

target_link_libraries(coverage_planning_node
  coverage_planning
)

ament_auto_package(
  INSTALL_TO_SHARE
  # config
  launch
)


