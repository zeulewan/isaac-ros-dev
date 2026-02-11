#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

python3 -c "
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=[
            ('visual_slam/image_0', 'front_stereo_camera/left/image_rect_color'),
            ('visual_slam/camera_info_0', 'front_stereo_camera/left/camera_info'),
            ('visual_slam/image_1', 'front_stereo_camera/right/image_rect_color'),
            ('visual_slam/camera_info_1', 'front_stereo_camera/right/camera_info'),
        ],
        parameters=[{
            'use_sim_time': True,
            'enable_image_denoising': True,
            'rectified_images': True,
            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': True,
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
        }],
    )

    container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([container])

from launch import LaunchService
ls = LaunchService()
ls.include_launch_description(generate_launch_description())
ls.run()
"
