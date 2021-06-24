FROM osrf/ros:noetic-desktop-full

ARG GIT_COMMIT=unknown
LABEL git-commit=$GIT_COMMIT
ARG CI_JOB_TOKEN

ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO=noetic
ENV ROBOGYM_WS=/robogym_ws

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
  apt-get update && apt-get install -y \
  apt-utils build-essential psmisc vim-gtk \
  git swig sudo libcppunit-dev \
  python3-catkin-tools python3-rosdep python3-pip \
  python3-rospkg python3-future python3-osrf-pycommon

RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
    mkdir -p $ROBOGYM_WS/src &&\
    cd $ROBOGYM_WS/src &&\
    git clone -b $ROS_DISTRO https://github.com/jr-robotics/mir_robot.git &&\
    git clone -b $ROS_DISTRO https://github.com/jr-robotics/universal_robot.git &&\ 
    # PANDA START
    # git clone -b v0.7.1-dev https://github.com/jr-robotics/franka_ros_interface &&\
    # git clone https://github.com/jr-robotics/franka_panda_description &&\ 
    # git clone -b ${ROS_DISTRO}-devel https://github.com/jr-robotics/panda_simulator &&\
    # git clone https://github.com/orocos/orocos_kinematics_dynamics &&\
    # cd orocos_kinematics_dynamics && git checkout b35c424e77ebc5b7e6f1c5e5c34f8a4666fbf5bc && cd .. &&\
    # PANDA END
    # git clone https://github.com/jr-robotics/Universal_Robots_ROS_Driver.git 
    # git clone https://gitlab-ci-token:${CI_JOB_TOKEN}@robotics-git.joanneum.at/perception/scene_perception.git 
    cd $ROBOGYM_WS &&\
    apt-get update &&\
    rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO --as-root=apt:false &&\
    catkin init &&\
    catkin build &&\
    pip3 install robo-gym-server-modules scipy numpy
    # Panda requirement
    # pip install --upgrade numpy numpy-quaternion==2020.5.11.13.33.35

ARG CACHEBUST=1

ADD . $ROBOGYM_WS/src/robo-gym-robot-servers

# Build ROS Workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
    source $ROBOGYM_WS/devel/setup.bash &&\ 
    cd $ROBOGYM_WS && \
    apt-get update && \
    # rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO --as-root=apt:false && \
    rosdep install --from-paths src/robo-gym-robot-servers/ur_robot_server src/robo-gym-robot-servers/mir100_robot_server -i -y --rosdistro $ROS_DISTRO --as-root=apt:false &&\
    # catkin build
    catkin build ur_robot_server mir100_robot_server

COPY ./ros-entrypoint.sh /
ENTRYPOINT ["/ros-entrypoint.sh"]

CMD ["bash"]
