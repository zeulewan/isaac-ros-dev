#!/bin/bash
# G1 auto-start services
# Runs on container startup via NVIDIA's entrypoint system

source /opt/ros/jazzy/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Foxglove bridge (visualization on port 8765)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &

# H.264 NVENC republisher (GPU camera compression)
# Uncomment when camera streaming is needed:
# ros2 run foxglove_compressed_video_transport republish --ros-args \
#   -r in/compressed:=/front_stereo_camera/left/image_raw/h264 \
#   -r out:=/front_stereo_camera/left/image_raw &

# Teleop joystick (Foxglove joystick -> /cmd_vel)
ros2 run teleop_twist_joy teleop_node --ros-args \
  -p require_enable_button:=false \
  -p axis_linear.x:=1 -p axis_angular.yaw:=0 \
  -p scale_linear.x:=-1.0 -p scale_angular.yaw:=1.0 &
