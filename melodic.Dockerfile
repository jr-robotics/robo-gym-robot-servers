FROM osrf/ros:melodic-desktop-full

ARG GIT_COMMIT=unknown
ENV GIT_COMMIT $GIT_COMMIT
LABEL git-commit=$GIT_COMMIT
ARG CI_JOB_TOKEN

ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO=melodic
ENV ROBOGYM_WS=/robogym_ws

RUN apt-get update && apt-get install -y \
  apt-utils build-essential psmisc vim-gtk \
  git swig sudo libcppunit-dev \
  python-catkin-tools python-rosdep python-pip \
  python-rospkg python-future

# Install pkgs required by Panda
RUN apt-get update && apt-get install -q -y \
    ros-${ROS_DISTRO}-rospy-message-converter \
    ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander \
    ros-${ROS_DISTRO}-moveit-visual-tools

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

COPY ./install-ros-dependencies.bash /
RUN chmod +x /install-ros-dependencies.bash && /install-ros-dependencies.bash
# Clone source of reqired ROS packages and install dependencies
# RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
#     mkdir -p $ROBOGYM_WS/src && \
#     cd $ROBOGYM_WS/src && \
#     git clone -b $ROS_DISTRO https://github.com/jr-robotics/mir_robot.git && \
#     git clone -b $ROS_DISTRO https://github.com/jr-robotics/universal_robot.git && \
#     # PANDA START
#     git clone -b v0.7.1-dev https://github.com/jr-robotics/franka_ros_interface && \
#     git clone https://github.com/jr-robotics/franka_panda_description && \
#     git clone -b ${ROS_DISTRO}-devel https://github.com/jr-robotics/panda_simulator && \
#     git clone https://github.com/orocos/orocos_kinematics_dynamics && \
#     cd orocos_kinematics_dynamics && git checkout b35c424e77ebc5b7e6f1c5e5c34f8a4666fbf5bc && cd .. && \
#     # PANDA END
#     # git clone https://github.com/jr-robotics/Universal_Robots_ROS_Driver.git && \
#     # git clone https://gitlab-ci-token:${CI_JOB_TOKEN}@robotics-git.joanneum.at/perception/scene_perception.git && \
#     cd $ROBOGYM_WS && \
#     apt-get update && \
#     rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO --as-root=apt:false && \
#     catkin init && \
#     catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo

RUN \
  pip install --upgrade pip && \
  pip install robo-gym-server-modules && \
  pip install scipy numpy && \
  # PANDA START
  pip install --upgrade numpy numpy-quaternion==2020.5.11.13.33.35
  # PANDA END

ARG CACHEBUST=1

ADD . $ROBOGYM_WS/src/robo-gym-robot-servers

# Build ROS Workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $ROBOGYM_WS && \
    apt-get update && \
    rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO --as-root=apt:false && \
    catkin init && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo

COPY ./ros-entrypoint.sh /
ENTRYPOINT ["/ros-entrypoint.sh"]

CMD ["bash"]
