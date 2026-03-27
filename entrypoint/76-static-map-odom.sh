#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom \
  --ros-args -p use_sim_time:=true &
