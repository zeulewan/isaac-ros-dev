#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
