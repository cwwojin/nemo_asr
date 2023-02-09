cmake_minimum_required(VERSION 2.8.3)
project(nemo_asr)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_package(
  DEPENDS rospy
#  INCLUDE_DIRS include
#  LIBRARIES nemo_asr
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)