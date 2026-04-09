#!/usr/bin/env python3
from launch import LaunchDescription
import launch.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('eyantrasim'), 'launch')


    spawn_eyantrasim_bots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'sim.launch.py')
        )
    )


    sim_controller = Node(
        package='eyantrasim',
        executable='sim_control.py',
        name='sim_controller'
    )

    rosbag = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', 'task_1a',
            '/eyantrasim/crystal/pose',
            '/eyantrasim/frostbite/pose',
            '/eyantrasim/glacio/pose'
        ],
        output='screen'
    )


    return LaunchDescription([
        spawn_eyantrasim_bots,
        sim_controller,
        rosbag

    ])
