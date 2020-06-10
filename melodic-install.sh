#!/bin/sh

# Run this from the src folder
echo "Installing robot-servers..."
cd ~/robogym_ws/src
git clone -b melodic https://github.com/jr-robotics/mir_robot.git
git clone -b melodic https://github.com/jr-robotics/universal_robot.git
git clone https://github.com/jr-robotics/Universal_Robots_ROS_Driver.git

cd ..
sudo apt-get update
sudo apt-get install -qq -y python-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src -i -y --rosdistro melodic
source /opt/ros/melodic/setup.bash
catkin init
catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo
echo "Installed robot-servers"
