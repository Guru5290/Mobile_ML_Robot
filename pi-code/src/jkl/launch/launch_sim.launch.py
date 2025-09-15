import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'jkl'

    # Load your ROS-2 control controllers YAML
    controllers_yaml = os.path.join(
        '/home/g/Mobile_ML_Robot/pi-code/src/jkl/config',
        'my_controllers.yaml'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml'),
            {'use_sim_time': True}
        ],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
        output='screen'
    )

    default_world = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.sdf')
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world, description='Path to the world SDF to load'
    )

    set_gz_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory(package_name), 'models')
    )

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v 4 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v 4 '}.items()
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
        arguments=['-topic', 'robot_description', '-name', 'jkl', '-z', '0.1'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output='screen'
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output='screen'
    )

    delayed_diff_spawner = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[diff_drive_spawner])
    )
    delayed_joint_spawner = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[joint_broad_spawner])
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file':os.path.join(get_package_share_directory(package_name), "config", "gz_bridge.yaml")}],
        output='screen'
    )
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output='screen'
    )
        # Specify the actions
    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=['/home/g/Mobile_ML_Robot/pi-code/src/jkl/config/ekf.yaml'] #  os.path.join(get_package_share_directory(package_name), "config", "ekf.yaml")
    )

    return LaunchDescription([
        rsp,
        # joystick,
        twist_mux,
        world_arg,
        set_gz_resources,
        gzserver_cmd,
        gzclient_cmd,
        ros2_control_node,          # → Added here
        spawn_entity,
        delayed_diff_spawner,
        delayed_joint_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge,
        ekf_localization
    ])
