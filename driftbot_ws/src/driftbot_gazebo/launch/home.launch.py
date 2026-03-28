"""Gazebo home world + robot spawn."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("driftbot_gazebo")
    pkg_desc   = get_package_share_directory("driftbot_description")

    world_file  = os.path.join(pkg_gazebo, "worlds", "home.sdf")
    models_path = os.path.join(pkg_gazebo, "models")
    xacro_file  = os.path.join(pkg_desc,   "urdf",   "robot.urdf.xacro")
    gz_launch   = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    # models path first
    if models_path:
        prev = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        tail = [p for p in prev.split(os.pathsep) if p and os.path.normpath(p) != os.path.normpath(models_path)]
        os.environ["GZ_SIM_RESOURCE_PATH"] = models_path if not tail else models_path + os.pathsep + os.pathsep.join(tail)

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=models_path,
            prepend=True,
        ),

        SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb"),

        DeclareLaunchArgument(
            "spawn_z", default_value="0.01",
            description="Spawn height (m).",
        ),
        DeclareLaunchArgument(
            "spawn_x",
            default_value="7.5",
            description="Spawn world x (m).",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            default_value="9.0",
            description="Spawn world y (m).",
        ),
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="-1.5708",
            description="Initial yaw (rad) about +Z.",
        ),
        DeclareLaunchArgument(
            "software_gl",
            default_value="false",
            description="mesa gl",
        ),

        # ogre2 lidar
        SetEnvironmentVariable(
            name="LIBGL_ALWAYS_SOFTWARE",
            value="1",
            condition=IfCondition(LaunchConfiguration("software_gl")),
        ),
        SetEnvironmentVariable(
            name="MESA_GL_VERSION_OVERRIDE",
            value="3.3",
            condition=IfCondition(LaunchConfiguration("software_gl")),
        ),
        SetEnvironmentVariable(
            name="MESA_GLSL_VERSION_OVERRIDE",
            value="330",
            condition=IfCondition(LaunchConfiguration("software_gl")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={
                "gz_args": (
                    f"-r -v 2 {world_file} "
                    "--render-engine ogre2 "
                    "--render-engine-gui ogre2 "
                    "--render-engine-server ogre2"
                ),
            }.items(),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_driftbot",
            arguments=[
                "-name", "driftbot",
                "-topic", "robot_description",
                "-x", LaunchConfiguration("spawn_x"),
                "-y", LaunchConfiguration("spawn_y"),
                "-z", LaunchConfiguration("spawn_z"),
                "-Y", LaunchConfiguration("spawn_yaw"),
            ],
            output="screen",
        ),

        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=["gz", "topic", "-t", "/magnet_off",
                         "-m", "gz.msgs.Empty", "-p", ""],
                    output="screen",
                )
            ],
        ),
    ])
