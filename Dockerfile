ARG ROS_DISTRO=melodic
FROM osrf/ros:$ROS_DISTRO-desktop-full

ARG ROS_DISTRO
ARG GIT_COMMIT=unknown
LABEL git-commit=$GIT_COMMIT
ARG CI_JOB_TOKEN

ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROBOGYM_WS=/robogym_ws

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

COPY ./install-ros-dependencies.bash /
RUN chmod +x /install-ros-dependencies.bash && /install-ros-dependencies.bash

ARG CACHEBUST=1

ADD . $ROBOGYM_WS/src/robo-gym-robot-servers

# Build ROS Workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $ROBOGYM_WS && \
    apt-get update && \
    rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO --as-root=apt:false && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo

COPY ./ros-entrypoint.sh /
ENTRYPOINT ["/ros-entrypoint.sh"]

CMD ["bash"]
