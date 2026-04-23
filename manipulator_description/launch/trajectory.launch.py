import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

 
def generate_launch_description():
    # Locate the package's share directory (where installed files live after colcon build)
    pkg_share = get_package_share_directory('manipulator_description')
 
    # Path to the xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'manipulator.urdf.xacro')
 
    # Path to the RViz config (may not exist yet — that's OK, RViz opens with defaults)
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
 
    # Process the xacro into a URDF string.
    # Command() runs at launch time: `xacro /path/to/file.xacro`
    robot_description = ParameterValue(Command(['xacro ', xacro_file]),value_type=str)
 
    return LaunchDescription([
 
        # --- robot_state_publisher ---
        # This node does two things:
        #   (a) Takes the URDF string as a parameter called 'robot_description'
        #   (b) Subscribes to /joint_states (sensor_msgs/JointState)
        #   (c) Publishes TF transforms for every link in the URDF
        # Without this, RViz wouldn't know where any link is.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
 
        # --- joint_state_publisher_gui ---
        # Opens a small window with a slider for each movable joint in your URDF.
        # When you drag a slider, it publishes a JointState message on /joint_states.
        # robot_state_publisher picks that up and updates TFs.
        # In Phase 2, we'll replace this with our own trajectory_publisher node.
        Node(
            package='manipulator_planning',
            executable='trajectory_publisher',
            output='screen',
        ),
 
        # --- RViz2 ---
        # The 3D visualiser. If the config file exists, it loads those settings.
        # If not, it opens with a blank scene and you add displays manually.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
        ),
    ])