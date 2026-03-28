"""Launch all nodes."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    task_share = get_package_share_directory('driftbot_task')
    gazebo_share = get_package_share_directory('driftbot_gazebo')

    args = [
        DeclareLaunchArgument('spawn_x', default_value='7.5'),
        DeclareLaunchArgument('spawn_y', default_value='9.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='-1.5708'),
        DeclareLaunchArgument('mission_id', default_value='1'),
    ]

    # Gazebo + spawn
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'home.launch.py')
        ),
        launch_arguments={
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
    )

    # ROS-GZ bridge
    bridge = TimerAction(period=8.0, actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            parameters=[
                {'config_file': os.path.join(
                    task_share, 'config', 'gz_bridge.yaml')},
                {'use_sim_time': True},
            ],
            output='screen',
        ),
    ])

    # Support + task
    stack = TimerAction(period=12.0, actions=[
        Node(
            package='driftbot_task',
            executable='gz_support',
            name='gz_support',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='driftbot_task',
            executable='task_node',
            name='task_node',
            parameters=[
                {'use_sim_time': True},
                {'mission_id': LaunchConfiguration('mission_id')},
                {'spawn_x': LaunchConfiguration('spawn_x')},
                {'spawn_y': LaunchConfiguration('spawn_y')},
                {'spawn_yaw': LaunchConfiguration('spawn_yaw')},
            ],
            output='screen',
        ),
    ])

    # RViz + camera
    viz = TimerAction(period=15.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', os.path.join(task_share, 'rviz', 'mission.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            remappings=[('image', '/camera/image_raw')],
            parameters=[{'use_sim_time': True}],
            additional_env={'QT_X11_NO_MITSHM': '1'},
        ),
    ])

    return LaunchDescription(args + [gazebo, bridge, stack, viz])
