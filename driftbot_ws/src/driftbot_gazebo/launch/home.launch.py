"""Launch the driftbot home world with the robot spawned inside it.

Usage:
  ros2 launch driftbot_gazebo home.launch.py

If the Gazebo window is blank, try:
  LIBGL_ALWAYS_SOFTWARE=1 ros2 launch driftbot_gazebo home.launch.py
"""
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

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        # Make Gazebo find our custom models via model:// URIs
        AppendEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=models_path),

        # WSL2: WAYLAND_DISPLAY + DISPLAY are both set. Qt defaults to Wayland,
        # but Ogre1 creates a GLX context on X11. Force xcb so both use X11
        # (via XWayland), eliminating the blank 3-D viewport.
        SetEnvironmentVariable(name="QT_QPA_PLATFORM",            value="xcb"),
        # Force Mesa software rasteriser (swrast / llvmpipe) so Ogre1 renders
        # without /dev/dxg GPU passthrough.
        SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE",      value="1"),
        SetEnvironmentVariable(name="MESA_GL_VERSION_OVERRIDE",   value="3.3"),
        SetEnvironmentVariable(name="MESA_GLSL_VERSION_OVERRIDE", value="330"),

        DeclareLaunchArgument(
            "spawn_z", default_value="0.01",
            description="Robot spawn z-height in metres.",
        ),

        # --render-engine ogre uses Ogre1 which works on WSL2 without GPU passthrough.
        # Ogre2 (default) requires GPU passthrough (/dev/dxg) which this system lacks.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": f"-r -v 2 {world_file} --render-engine ogre"}.items(),
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
                "-name",  "driftbot",
                "-topic", "robot_description",
                "-x",     "1.5",
                "-y",     "9.0",
                "-z",     LaunchConfiguration("spawn_z"),
            ],
            output="screen",
        ),

        # detach_on_start in the URDF is ignored by gz-sim 8.10.0 — the
        # DetachableJoint plugin attaches all objects at startup. Publishing to
        # /magnet_off via the TriggeredPublisher releases all 6 at once.
        # 5 s gives Gazebo and the robot-spawn node enough time to initialise.
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
