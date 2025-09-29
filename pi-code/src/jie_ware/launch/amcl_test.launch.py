from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('jkl')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Robot description
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(pkg_dir, 'urdf', 'wpb_home.urdf')
    )

    # Launch Gazebo with world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_dir, 'worlds', 'robocup_home.world')}.items()
    )

    # Example of spawning one object into Gazebo (repeat for others)
    spawn_bed = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'bed',
                   '-file', os.path.join(pkg_dir, 'models', 'bed.model'),
                   '-x', '5.0', '-y', '-3.9', '-z', '0', '-Y', '3.14159'],
        output='screen'
    )

    # Map server (Nav2)
    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': 'f1.yaml'}],
            remappings=remappings
        ),

    # AMCL (Nav2)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('wpb_home_tutorials'),
                                 'nav_lidar', 'amcl_omni.yaml')]
    )

    # Nav2 bringup (instead of move_base)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(get_package_share_directory('wpb_home_tutorials'),
                                        'nav_lidar', 'nav2_params.yaml')
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'nav.rviz')],
        output='screen'
    )

    return LaunchDescription([
        # model_arg,
        # gazebo,
        # spawn_bed,
        map_server,
        amcl,
        # nav2_bringup,
        # robot_state_publisher,
        # rviz
    ])
