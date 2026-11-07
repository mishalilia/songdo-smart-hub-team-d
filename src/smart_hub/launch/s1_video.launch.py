# Author: Andrey Li (12250249)
"""
S1 on a recorded video: video_source plays the file, passenger_detector counts in camera mode.

    ros2 launch smart_hub s1_video.launch.py video:=/path/to/vtest.avi
    ros2 topic echo /hub/queue
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video = LaunchConfiguration('video')
    return LaunchDescription([
        DeclareLaunchArgument('video', description='path of the video file'),
        Node(package='smart_hub', executable='video_source', name='video_source',
             parameters=[{'video_path': video}]),
        Node(package='smart_hub', executable='passenger_detector', name='passenger_detector',
             parameters=[{'input_mode': 'camera', 'image_topic': '/camera/video'}]),
    ])
