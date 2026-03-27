#!/bin/bash
# Launch Nav2 autonomous navigation stack inside the Isaac ROS container
# Uses cuVSLAM for localization (no AMCL/map_server needed)
# Usage: docker exec -it isaac_ros_dev_container bash /workspaces/isaac_ros-dev/launch_nav2.sh

export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch /workspaces/isaac_ros-dev/nav2_carter_launch.py \
  params_file:=/workspaces/isaac_ros-dev/nav2_carter_params.yaml \
  use_sim_time:=True \
  autostart:=True
