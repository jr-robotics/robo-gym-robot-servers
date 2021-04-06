FROM osrf/ros:melodic-desktop-full

ARG GIT_COMMIT=unknown
ENV GIT_COMMIT $GIT_COMMIT
LABEL git-commit=$GIT_COMMIT
ARG CI_JOB_TOKEN

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
  apt-utils build-essential psmisc vim-gtk \
  python-catkin-tools python-rosdep python-pip

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

ENV ROS_DISTRO=melodic
ENV ROBOGYM_WS=/robogym_ws

ADD . $ROBOGYM_WS/src/robo-gym-robot-servers

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p $ROBOGYM_WS/src && \
    cd $ROBOGYM_WS/src && \
    git clone -b $ROS_DISTRO https://github.com/jr-robotics/mir_robot.git && \
    git clone -b $ROS_DISTRO https://github.com/jr-robotics/universal_robot.git && \
    # git clone https://github.com/jr-robotics/Universal_Robots_ROS_Driver.git && \
    # git clone https://gitlab-ci-token:${CI_JOB_TOKEN}@robotics-git.joanneum.at/perception/scene_perception.git && \
    cd $ROBOGYM_WS && \
    apt-get update && \
    rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO --as-root=apt:false && \
    catkin init && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo

RUN \
  pip install --upgrade pip && \
  pip install robo-gym-server-modules && \
  pip install scipy numpy

COPY ./melodic-entrypoint.sh /

ENTRYPOINT ["/melodic-entrypoint.sh"]

CMD ["bash"]
