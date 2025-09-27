import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    AppendEnvironmentVariable,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'jkl'

    controllers_yaml = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # Twist mux
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    # World file
    default_world = os.path.join(get_package_share_directory(package_name), 'worlds', 'GF.sdf')
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world, description='Path to the world SDF to load'
    )

    # Gazebo resource path (models)
    set_gz_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory(package_name), 'models')
    )

    # Gazebo server and client (start server unpaused)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v4 ', 
            'on_exit_shutdown': 'true'
            }.items()
    )

    # Controller Manager node with YAML parameters
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'knight',
            '-x', '-2.5',
            '-y', '1.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ]
    )
    delayed_spawn = TimerAction(period=1.0, actions=[spawn_entity])

    # Controller spawners (explicit controller_manager, staggered, longer timeout)
    joint_broad_spawner = TimerAction(
        period=1.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_broad",
                "--controller-manager", "/controller_manager",   # in-Gazebo manager
                "--controller-manager-timeout", "30",
                "--activate"
            ],
            output='screen'
        )]
    )

    diff_drive_spawner = TimerAction(
        period=1.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diff_cont",
                "--controller-manager", "/controller_manager",   # in-Gazebo manager
                "--controller-manager-timeout", "30",
                "--activate"
            ],
            output='screen'
        )]
    )

    # Bridges
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={os.path.join(get_package_share_directory(package_name), "config", "gz_bridge.yaml")}'
        ],
        output='screen'
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output='screen'
    )
    
    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(package_name), "config", "ekf.yaml"),
                    {'use_sim_time': True}]   
    )

    return LaunchDescription([
        world_arg,
        set_gz_resources,
        rsp,
        # joystick,
        twist_mux,
        gzserver_cmd,
        gzclient_cmd,
        ros2_control_node,
        delayed_spawn,
        joint_broad_spawner,
        diff_drive_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge,
        ekf_localization
    ])
