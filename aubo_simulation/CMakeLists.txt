cmake_minimum_required(VERSION 3.0.2)
project(aubo_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
                                roscpp
                                std_msgs
                                robot_state_publisher
                            )
catkin_package(CATKIN_DEPENDS   roscpp     )
include_directories(include   ${catkin_INCLUDE_DIRS})
add_library(aubo_simulation    src/test.cpp  src/kinematics.cpp )
add_executable(test2  src/test.cpp src/kinematics.cpp )
target_link_libraries(test2  ${catkin_LIBRARIES})
                          
