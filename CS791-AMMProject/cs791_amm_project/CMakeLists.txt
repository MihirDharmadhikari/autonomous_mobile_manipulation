cmake_minimum_required(VERSION 3.0.2)
project(cs791_amm_project)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_install()
cs_export(LIBRARIES)
