cmake_minimum_required(VERSION 3.5)
project(avp_planning)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC -Wno-dev")

add_definitions(-DDATASET_PATH="${PROJECT_SOURCE_DIR}/map/")
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=1) # for clangd

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf lgsvl_msgs)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
    /usr/include/eigen3
)

add_executable(avp_planning 
    src/avp_planning_node.cpp
    src/avp_map.cpp
    src/slot.cpp
    src/ros_viewer.cpp
    src/planner.cpp
    src/controller.cpp
    src/KinematicModel.cpp
)

target_link_libraries(avp_planning
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)