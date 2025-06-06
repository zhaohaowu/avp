cmake_minimum_required(VERSION 3.5)
project(ipm)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

add_definitions(-DPARAMS="${PROJECT_SOURCE_DIR}/params/")
add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=1) # for clangd

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs tf)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})

add_executable(ipm src/ipm.cc src/ipm_node.cc)

target_link_libraries(ipm ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})