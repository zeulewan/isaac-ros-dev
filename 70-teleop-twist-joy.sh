#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash

# Start teleop_twist_joy (converts /joy → /cmd_vel)
# Foxglove joystick only publishes when touched, so no zero-flooding.
# Do NOT seed /joy with zeros — that overrides Nav2.
ros2 run teleop_twist_joy teleop_node --ros-args \
  -p require_enable_button:=false \
  -p axis_linear.x:=1 \
  -p axis_angular.yaw:=0 \
  -p scale_linear.x:=-1.0 \
  -p scale_angular.yaw:=1.0 &
