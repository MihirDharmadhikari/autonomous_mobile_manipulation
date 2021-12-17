cmake_minimum_required(VERSION 3.0.2)
project(mm_coverage_planner)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
)
catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    moveit_visual_tools
)

cs_add_library(
  ${PROJECT_NAME}
  src/utils/utils.cpp
  src/utils/data_loader.cpp
  src/viewpoint_grouper.cpp
  src/path_planner.cpp
)

cs_add_executable(planner_node src/main.cpp)
target_link_libraries(planner_node ${PROJECT_NAME})

cs_install()
cs_export(LIBRARIES)
