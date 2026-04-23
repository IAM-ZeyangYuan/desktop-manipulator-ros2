import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('manipulator_description')
    xacro_path = os.path.join(pkg_description, 'urdf', 'manipulator.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_path]),value_type=str)
    rviz_config = os.path.join(pkg_description, 'rviz', 'config.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='manipulator_planning',
            executable='trajectory_service',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])