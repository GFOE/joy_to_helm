cmake_minimum_required(VERSION 2.8.3)
project(joy_to_helm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  marine_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES appcast_view
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

