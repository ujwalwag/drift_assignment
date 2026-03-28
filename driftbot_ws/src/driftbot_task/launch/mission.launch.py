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

"""

Launch Gazebo home world, ros_gz_bridge, support node, RViz, camera, mission.

Spawn pose defaults (7.5, 9, yaw −π/2) are forwarded to home.launch and driftbot_task
(spawn_x/y/yaw must stay consistent so odom→world mapping in task_node is correct).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('driftbot_gazebo')
    pkg_task = get_package_share_directory('driftbot_task')

    home_launch = os.path.join(pkg_gazebo, 'launch', 'home.launch.py')
    rviz_cfg = os.path.join(pkg_task, 'rviz', 'mission.rviz')
    bridge_yaml = os.path.join(pkg_task, 'config', 'gz_bridge.yaml')
    mission_yaml = os.path.join(pkg_task, 'config', 'mission.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_x',
            default_value='7.5',
            description='World spawn x (m); forwarded to driftbot_task spawn_x.',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='9.0',
            description='World spawn y (m); forwarded to driftbot_task spawn_y.',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='-1.5708',
            description='Initial yaw (rad); reference −π/2 unless overridden.',
        ),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(home_launch),
            launch_arguments={
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            }.items(),
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='bridge_node',
                    name='ros_gz_bridge',
                    output='screen',
                    parameters=[
                        {'config_file': bridge_yaml, 'use_sim_time': True},
                    ],
                )
            ],
        ),
        TimerAction(
            period=1.2,
            actions=[
                Node(
                    package='driftbot_task',
                    executable='gz_bridge_support',
                    name='gz_bridge_support',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
                Node(
                    package='driftbot_task',
                    executable='task_node',
                    name='driftbot_task',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'mission_config': mission_yaml},
                        {
                            'spawn_x': ParameterValue(
                                LaunchConfiguration('spawn_x'), value_type=str
                            ),
                        },
                        {
                            'spawn_y': ParameterValue(
                                LaunchConfiguration('spawn_y'), value_type=str
                            ),
                        },
                        {
                            'spawn_yaw': ParameterValue(
                                LaunchConfiguration('spawn_yaw'), value_type=str
                            ),
                        },
                    ],
                ),
            ],
        ),
        TimerAction(
            period=2.5,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_cfg],
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'image_view', 'image_view',
                        '--ros-args', '-p', 'use_sim_time:=true',
                        '-r', 'image:=/camera/image_raw',
                    ],
                    output='screen',
                    shell=False,
                ),
            ],
        ),
    ])
