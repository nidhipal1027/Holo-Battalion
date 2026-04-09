#!/usr/bin/env python3
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace='eyantrasim', package='eyantrasim',
            executable='eyantrasim_node', output='screen'),
    ])