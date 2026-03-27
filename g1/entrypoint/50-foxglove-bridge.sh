#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
