#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/catkin_ws/devel/setup.bash"
roslaunch canopy_client canopy_leaflet.launch host:=$HOST port:=$PORT key:=$KEY container_name:=$NAME &
exec "$@"
