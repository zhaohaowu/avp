cmake_minimum_required(VERSION 3.5)
project(avp_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC -Wno-dev")

add_definitions(-DDATASET_PATH="${PROJECT_SOURCE_DIR}/map/")
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=1) # for clangd

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs tf cv_bridge)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
    /usr/include/eigen3
)

add_executable(avp_mapping
    src/avp_mapping_node.cc
    src/map.cc
    src/slot.cc
)

target_link_libraries(avp_mapping
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)