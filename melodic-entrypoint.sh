#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
source /robogym_ws/devel/setup.bash
echo "robo-gym-robot-servers commit SHA: $GIT_COMMIT"


exec "$@"
