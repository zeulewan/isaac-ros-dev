#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
ros2 launch /workspaces/isaac_ros-dev/nav2_carter_launch.py \
  params_file:=/workspaces/isaac_ros-dev/nav2_carter_params.yaml \
  use_sim_time:=True \
  autostart:=True &
