cmake_minimum_required(VERSION 3.0.2)
project(base_planner)

add_compile_options(-std=c++14)

find_package(GTest REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    visualization_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS geometry_msgs nav_msgs visualization_msgs
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} INTERFACE )

install(
    DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
