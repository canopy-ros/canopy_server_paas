#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/catkin_ws/devel/setup.bash"
cd ~/catkin_ws/src/canopy_client
git pull
roslaunch canopy_client canopy_leaflet.launch host:=$HOST port:=$PORT key:=$KEY container_name:=$NAME &
exec "$@"
