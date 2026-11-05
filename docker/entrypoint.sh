#!/bin/sh
# The container's entrypoint: every command runs with ROS 2 and the px4_msgs workspace, which sets up
# ROS 2 itself, sourced.
set -e
. /opt/px4_ws/install/setup.sh
exec "$@"
