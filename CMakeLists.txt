cmake_minimum_required(VERSION 3.0.2)
project(controller_n)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(controller_n
  src/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(controller_n
  ${catkin_LIBRARIES}
)
