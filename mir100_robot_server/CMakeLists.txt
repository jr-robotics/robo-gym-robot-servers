cmake_minimum_required(VERSION 2.8.3)
project(mir100_robot_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS scripts/cmd_vel_command_handler.py 
                               scripts/robot_pose_publisher.py
                               scripts/robot_server.py
                               scripts/ros_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})