import os
from launch import LaunchDescription #every launch file returns a Launch Description object
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit #a specific event handler that fires when a process exits
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue 
from launch.substitutions import Command


def generate_launch_description(): #every Python launch file must have this function
    pkg = get_package_share_directory('manipulator_description') #gets the path of your installed package files
    xacro_path = os.path.join(pkg, 'urdf', 'manipulator.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str) # the Command returns the output of the command in a string
    #ParameterValue wraps the results so that it can be passed in as a ROS2 parameter
    world_path = os.path.join(pkg, 'worlds', 'desktop.sdf')
    rviz_config = os.path.join(pkg, 'rviz', 'config.rviz')

    gazebo = IncludeLaunchDescription( #this lets you to call another launch file from here
        PythonLaunchDescriptionSource( #this tells IncludeLaunchDescription that you are including a Python file, not a xml or yaml launch files
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(), #-r means immediately f makes the whole thing a f string
        #.items() is a dictionary method that returns key-value pairs
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': True},
        ],
    )

    #it reads the urdf from the robot_description topic, converts it to sdf, sends
    #it to the Gazebo server via a Gazebo service call, the robot then spawns in the Gazebo world
    #with a name manipulator
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'manipulator',
        ],
        output='screen',
    )

    # spawn controllers after robot is spawned
    #the --param-file tells it where to find the controllers's configuration
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file',
                   os.path.join(pkg, 'config', 'controllers.yaml')],
    )

    spawn_jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--param-file',
                   os.path.join(pkg, 'config', 'controllers.yaml')],
    )

    # delay controller spawning until after robot spawn completes
    delayed_controllers = RegisterEventHandler( #registers a callbackk that fires when some event happens
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[spawn_jsb, spawn_jtc],
        )
    )

    # bridge clock from Gazebo to ROS2 (one directional)
    #syntax: TopicName@ROS2TYpe[GzType]: takes the gazebo topic /clock (which uses gz.msgs.Clock format)
    # convert each message to rosgraph_msgs/msgClock format
    #then republishes it on the ROS2 /clock topic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        delayed_controllers,
        bridge,
        rviz,
    ])