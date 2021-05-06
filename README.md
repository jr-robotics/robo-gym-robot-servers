# robo-gym-robot-servers

Repository containing Robot Servers ROS packages

The packages have been tested for ROS Melodic (recommended).



*WARNING for ROS Kinetic users*: 

The compatibility of the Universal Robots environments with ROS Kinetic has been maintained until version [v0.1.8](https://github.com/jr-robotics/robo-gym-robot-servers/tree/v0.1.8) included. After this version we integrated a refactored version of the [universal_robot repository](https://github.com/jr-robotics/universal_robot) which is not compatible with ROS Kinetic. To use the UR environments on ROS kinetic you need to use v0.1.8 of the robo-gym package and v.0.1.8 of the robo-gym-robot-servers. See [#16](https://github.com/jr-robotics/robo-gym/issues/16) for more details. 

### Robots currently implemented
- MiR100
- Universal Robots: UR3, UR3e, UR5, UR5e, UR10, UR10e, UR16


# Installation

## Ubuntu 20.04 - ROS Noetic

1. Install the required packages
```sh
sudo apt-get update && sudo apt-get install apt-utils build-essential psmisc vim-gtk git swig sudo libcppunit-dev python3-catkin-tools python3-rosdep python3-pip python3-rospkg python3-future python3-osrf-pycommon
```

2. Open a new terminal and set the environment variables. Use the same terminal for all the installation steps. 
```sh
# Set robo-gym ROS workspace folder
export ROBOGYM_WS=~/robogym_ws 
# Set ROS distribution
export ROS_DISTRO=noetic
```

3. Create a workspace folder in the home folder of your PC and clone this repository
```sh
mkdir -p $ROBOGYM_WS/src && cd $ROBOGYM_WS/src && git clone https://github.com/jr-robotics/robo-gym-robot-servers.git
```

4.  Setup your computer to accept software from packages.ros.org
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

5. Clone required packages, build the workspace and install required python modules
```sh
source /opt/ros/$ROS_DISTRO/setup.bash &&\
git clone -b $ROS_DISTRO https://github.com/jr-robotics/mir_robot.git &&\
#git clone -b $ROS_DISTRO https://github.com/jr-robotics/universal_robot.git &&\ 
cd $ROBOGYM_WS &&\
sudo apt-get update &&\
sudo rosdep init && rosdep update &&\
rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO &&\
catkin init &&\
catkin build &&\
pip3 install robo-gym-server-modules scipy numpy
```

## Ubuntu 18.04 - ROS Melodic 
<details>
<summary>Click to expand</summary>
<p>

1. Install the required packages
```sh
sudo apt-get update && sudo apt-get install apt-utils build-essential psmisc vim-gtk git swig sudo libcppunit-dev python-catkin-tools python-rosdep python-pip python-rospkg python-future
```

2. Open a new terminal and set the environment variables. Use the same terminal for all the installation steps. 
```sh
# Set robo-gym ROS workspace folder
export ROBOGYM_WS=~/robogym_ws 
# Set ROS distribution
export ROS_DISTRO=melodic
```

3. Create a workspace folder in the home folder of your PC and clone this repository
```sh
mkdir -p $ROBOGYM_WS/src && cd $ROBOGYM_WS/src && git clone https://github.com/jr-robotics/robo-gym-robot-servers.git
```

4.  Setup your computer to accept software from packages.ros.org
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

5. Clone required packages, build the workspace and install required python modules
```sh
source /opt/ros/$ROS_DISTRO/setup.bash &&\
git clone -b $ROS_DISTRO https://github.com/jr-robotics/mir_robot.git &&\
git clone -b $ROS_DISTRO https://github.com/jr-robotics/universal_robot.git &&\ 
cd $ROBOGYM_WS &&\
sudo apt-get update &&\
sudo rosdep init && rosdep update &&\
rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO &&\
catkin init &&\
catkin build &&\
pip install --upgrade pip &&\
pip install robo-gym-server-modules scipy numpy
```

6. Add the sourcing of ROS and the ROS workspace to your `.bashrc` file:
```sh
printf "source /opt/ros/$ROS_DISTRO/setup.bash\nsource $ROBOGYM_WS/devel/setup.bash" >> ~/.bashrc
```

</p>
</details>  

## Troubleshooting

For problems with step 2 and 3 refer to http://wiki.ros.org/melodic/Installation/Ubuntu.

# How to use

## MiR100

### Simulated Robot
Simulated Robot Servers are handled by the Server Manager. If you want to manually start a Simulated Robot Server use:
```
roslaunch mir100_robot_server sim_robot_server.launch gui:=true
```
### Real Robot

- Connect to the robot's network

In a terminal window:
- Set ROS Master IP: `export ROS_MASTER_URI=http://192.168.12.20:11311`
- Launch MiR100 Robot Server `roslaunch mir100_robot_server real_robot_server.launch gui:=true`


## Universal Robots

### Simulated Robot
Simulated Robot Servers are handled by the Server Manager. If you want to manually start a Simulated Robot Server use:
```
roslaunch ur_robot_server ur_robot_server.launch ur_model:=ur10  gui:=true
```

### Real Robot Server
#### Install UR ROS Driver

To control the UR Robots we use the new [UR ROS Driver](https://github.com/jr-robotics/Universal_Robots_ROS_Driver).
At the current status the [UR ROS Driver](https://github.com/jr-robotics/Universal_Robots_ROS_Driver) and the [Universal_robot](https://github.com/jr-robotics/universal_robot) package use two different robot descriptions, for this reason it is needed to setup the UR ROS Driver in a separate workspace to avoid conflicts between the two packages.

```bash
# Source ROS Melodic
source /opt/ros/melodic/setup.bash
# Source ROS Kinetic
# source /opt/ros/kinetic/setup.bash

# Create a new folder for the workspace
mkdir -p ~/urdriver_ws/src
cd ~/urdriver_ws
catkin init

# Clone the necessary packages
cd ~/urdriver_ws/src
git clone https://github.com/jr-robotics/Universal_Robots_ROS_Driver.git
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git

# Install dependencies
cd ~/urdriver_ws
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build the workspace
catkin build

```

For additional instructions on how to setup the driver on the robot follow the README of the [UR ROS Driver](https://github.com/jr-robotics/Universal_Robots_ROS_Driver).

#### How to use

*NOTE:* The following instructions and command lines have been written for the UR 10 but they apply to all the supported UR robots, for instance for using the UR 5 Robot Server it is sufficient to replace `ur10` with `ur5` in all the following command lines.


- Connect to the robot's network

In a terminal window start the UR ROS driver:

```bash
# Source ROS Melodic
source /opt/ros/melodic/setup.bash
# Source ROS Kinetic
# source /opt/ros/kinetic/setup.bash

# source the UR ROS Driver workspace
source ~/urdriver_ws/devel/setup.bash

# start the Driver (replace with the IP of your robot)
roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.12.70
```



In another terminal window start the Robot Server

```bash
# Source ROS Melodic
source /opt/ros/melodic/setup.bash
# Source ROS Kinetic
# source /opt/ros/kinetic/setup.bash

# source robo-gym workspace
source ~/robogym_ws/devel/setup.bash

# start the Robot Server
roslaunch ur_robot_server ur_robot_server.launch ur_model:=ur10 real_robot:=true gui:=true max_torque_scale_factor:=0.5 max_velocity_scale_factor:=0.5 speed_scaling:=0.5
```
