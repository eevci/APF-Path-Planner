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
set(SRC_FILES
        src/common/types/vector_2D.cpp
)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
