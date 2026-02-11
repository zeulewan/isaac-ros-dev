#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml
source /opt/ros/jazzy/setup.bash
source /tmp/m-explore-ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run explore_lite explore --ros-args \
  -p use_sim_time:=true \
  -p robot_base_frame:=base_link \
  -p costmap_topic:=/global_costmap/costmap \
  -p costmap_updates_topic:=/global_costmap/costmap_updates \
  -p visualize:=true \
  -p planner_frequency:=0.25 \
  -p progress_timeout:=30.0 \
  -p potential_scale:=3.0 \
  -p gain_scale:=1.0 \
  -p transform_tolerance:=0.5 \
  -p min_frontier_size:=1.5 \
  -p return_to_init:=false \
  --remap /tf:=tf --remap /tf_static:=tf_static
