# Copyright 2025 ujwalwag
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Sim + task stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch all."""
    pkg_gazebo = get_package_share_directory('driftbot_gazebo')
    pkg_task = get_package_share_directory('driftbot_task')

    home_launch = os.path.join(pkg_gazebo, 'launch', 'home.launch.py')
    rviz_cfg = os.path.join(pkg_task, 'rviz', 'tidying.rviz')
    bridge_yaml = os.path.join(pkg_task, 'config', 'tidying_bridge.yaml')
    tidying_yaml = os.path.join(pkg_task, 'config', 'tidying.yaml')

    # bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_yaml,
                'use_sim_time': True,
            },
        ],
    )

    sim_tf_relay = Node(
        package='driftbot_task',
        executable='sim_tf_relay',
        name='sim_tf_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    scan_costmap = Node(
        package='driftbot_task',
        executable='scan_costmap',
        name='scan_costmap',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    scan_relay = Node(
        package='driftbot_task',
        executable='scan_relay',
        name='scan_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    camera_view = Node(
        package='image_view',
        executable='image_view',
        name='camera_image_view',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('image', '/camera/image_raw')],
    )

    task_node = Node(
        package='driftbot_task',
        executable='task_node',
        name='driftbot_task',
        parameters=[
            {'use_sim_time': True},
            {'tidy_config': tidying_yaml},
        ],
        output='screen',
    )

    # t+1s bridge; t+1.2s nodes; t+2.5s rviz
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(home_launch)),
        TimerAction(period=1.0, actions=[gz_bridge]),
        TimerAction(
            period=1.2,
            actions=[sim_tf_relay, scan_relay, scan_costmap, task_node],
        ),
        TimerAction(period=2.5, actions=[rviz, camera_view]),
    ])
