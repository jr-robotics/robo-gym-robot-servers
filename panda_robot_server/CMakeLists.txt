cmake_minimum_required(VERSION 2.8.3)
project(panda_robot_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/joint_trajectory_command_handler.py 
                               scripts/robot_server.py
                               scripts/ros_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})