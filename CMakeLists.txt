cmake_minimum_required(VERSION 2.8.3)
project(uarm)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
