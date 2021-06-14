cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(panda_robot_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_python_setup()
catkin_install_python(PROGRAMS scripts/joint_trajectory_command_handler.py 
                               scripts/robot_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})