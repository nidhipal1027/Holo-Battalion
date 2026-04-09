#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    hb_perception_node = Node(
        package='hb_testing',
        executable='/usr/bin/python3',
        name='hb_perception',
        output='screen',
        arguments=[os.path.join(get_package_share_directory('hb_testing'), 'src', 'camera_testing.py')]
    )

    ld = LaunchDescription()
    ld.add_action(hb_perception_node)
    return ld