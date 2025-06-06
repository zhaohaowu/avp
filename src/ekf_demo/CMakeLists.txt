cmake_minimum_required(VERSION 3.5)
project(ekf_demo)

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=1) # for clangd

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ekf_demo src/ekf_demo.cc)

target_link_libraries(ekf_demo ${catkin_LIBRARIES})