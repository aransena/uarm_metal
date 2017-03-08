cmake_minimum_required(VERSION 2.8.3)
project(uarm_metal)

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


add_message_files(
   FILES
   Position.msg
   JointAngles.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)
