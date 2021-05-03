#!/bin/sh

source /opt/ros/$ROS_DISTRO/setup.bash 
mkdir -p $ROBOGYM_WS/src 
cd $ROBOGYM_WS/src 
git clone -b $ROS_DISTRO https://github.com/jr-robotics/mir_robot.git 
git clone -b $ROS_DISTRO https://github.com/jr-robotics/universal_robot.git 
# PANDA START
git clone -b v0.7.1-dev https://github.com/jr-robotics/franka_ros_interface 
git clone https://github.com/jr-robotics/franka_panda_description 
git clone -b ${ROS_DISTRO}-devel https://github.com/jr-robotics/panda_simulator 
git clone https://github.com/orocos/orocos_kinematics_dynamics 
cd orocos_kinematics_dynamics && git checkout b35c424e77ebc5b7e6f1c5e5c34f8a4666fbf5bc && cd .. 
# PANDA END
# git clone https://github.com/jr-robotics/Universal_Robots_ROS_Driver.git 
# git clone https://gitlab-ci-token:${CI_JOB_TOKEN}@robotics-git.joanneum.at/perception/scene_perception.git 
cd $ROBOGYM_WS 
apt-get update 
rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO --as-root=apt:false 
catkin init 
catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo
