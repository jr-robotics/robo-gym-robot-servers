#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash
source /robogym_ws/devel/setup.bash
echo "robo-gym-robot-servers commit SHA: $GIT_COMMIT"

exec "$@"
