import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg = get_package_share_directory('manipulator_description')
    xacro_path = os.path.join(pkg, 'urdf', 'manipulator.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)
    world_path = os.path.join(pkg, 'worlds', 'desktop.sdf')

    # 1. Start Gazebo with your world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 2. Spawn the robot into Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'manipulator',
            '-z', '0.0',
        ],
        output='screen',
    )

    # 3. robot_state_publisher (reads URDF, publishes TF)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
    ])