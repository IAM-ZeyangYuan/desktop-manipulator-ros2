import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg_description = get_package_share_directory('manipulator_description')
    xacro_path = os.path.join(pkg_description, 'urdf', 'manipulator.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)
    controllers_yaml = os.path.join(pkg_description, 'config', 'controllers.yaml')
    rviz_config = os.path.join(pkg_description, 'rviz', 'config.rviz')

    # controller_manager loads the hardware interface + controllers
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        output='screen',
    )

    # robot_state_publisher for TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # spawn controllers — delay slightly so controller_manager is ready
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    spawn_jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )

    # delay the controller spawning to give controller_manager time to start
    delayed_spawners = TimerAction(
        period=2.0,
        actions=[spawn_jsb, spawn_jtc],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        controller_manager_node,
        robot_state_publisher_node,
        delayed_spawners,
        rviz_node,
    ])