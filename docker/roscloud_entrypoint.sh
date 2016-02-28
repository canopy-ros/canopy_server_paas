#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/catkin_ws/devel/setup.bash"
roslaunch roscloud_client roscloud_server.launch host:=$HOST port:=$PORT key:=$HOSTNAME &
exec "$@"