#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"
echo "Sourced ROS 2 ${ROS_DISTRO}"

if [ -f /dnode/install/setup.bash ]
then
    source /enterprise_ws/install/setup.bash
    echo "Sourced dNode API workspace"
fi

# # source "/nav2_ws/install/setup.bash"
# export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/opt/ros/${my_release}/share/turtlebot3_gazebo/models

exec "$@"