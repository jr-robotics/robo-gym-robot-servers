cmake_minimum_required(VERSION 2.8.3)
project(mir100_robot_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()
