# robo-gym-robot-servers

Repository containing Robot Servers ROS packages

### Robots currently implemented
- MiR100
- Universal Robots UR 10


### Installation

Create a workspace folder in the home folder of your PC and clone this repository

```
mkdir -p ~/robogym_ws/src && cd ~/robogym_ws/src && git clone https://github.com/jr-robotics/robo-gym-robot-servers.git
```

Run the installation script
```
~/robogym_ws/src/robo-gym-robot-servers/install.sh
```
*NOTE*: Remember to source the workspace in every new shell before starting the Server Manager.

To source the workspace use:
```
source ~/robogym_ws/devel/setup.bash
```

### How to use

#### MiR100

##### Simulated Robot
Simulated Robot Servers are handled by the Server Manager. If you want to manually start a Simulated Robot Server use:
```
roslaunch mir100_robot_server sim_robot_server.launch gui:=true
```
##### Real Robot

- Connect to the robot's network

In a terminal window:
- Set ROS Master IP: `export ROS_MASTER_URI=http://192.168.12.20:11311`
- Launch MiR100 Robot Server `roslaunch mir100_robot_server real_robot_server.launch gui:=true`


#### UR10

##### Simulated Robot
Simulated Robot Servers are handled by the Server Manager. If you want to manually start a Simulated Robot Server use:
```
roslaunch ur_robot_server ur10_sim_robot_server.launch gui:=true
```

##### Real Robot Server

- Connect to the robot's network

In a terminal window start the ROS driver:
- Set ROS Master IP: `export ROS_MASTER_URI=http://192.168.12.20:11311`
- Check the IP of the PC on which the UR ROS Driver will be running with `ifconfig`
- Set the PC IP `export ROS_IP=192.168.12.192` (replace with IP of the PC)
- Launch the driver `roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.12.70` (replace with IP of the robot)

In another terminal window start the Robot Server
- Set ROS Master Ip: `export ROS_MASTER_URI=http://192.168.12.20:11311`
- Launch UR 10 Robot Server `roslaunch ur_robot_server ur10_real_robot_server.launch gui:=true max_torque_scale_factor:=0.5 max_velocity_scale_factor:=0.5 speed_scaling:=0.5`
