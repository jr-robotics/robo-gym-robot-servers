#!/bin/sh

# Run this from the src folder
echo "Installing robot-servers..."
cd ~/robogym_ws/src
git clone -b kinetic https://github.com/jr-robotics/mir_robot.git
git clone -b kinetic https://github.com/jr-robotics/universal_robot.git

cd ..
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -qq -y python-rosdep python-catkin-tools
sudo rosdep init
rosdep update
rosdep install --from-paths src -i -y --rosdistro kinetic
source /opt/ros/kinetic/setup.bash
catkin init
catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo
echo "Installed robot-servers"
