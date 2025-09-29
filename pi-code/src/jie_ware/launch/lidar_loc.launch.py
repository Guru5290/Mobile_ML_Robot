from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('jkl')
    default_map_yaml = os.path.abspath(os.path.join(pkg_dir, '../../../../f1.yaml'))
    map_yaml = LaunchConfiguration('map', default=default_map_yaml)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # Map server node
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, "yaml_filename": map_yaml}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Lidar node
    jie_ware_node = Node(
        package='jie_ware',
        executable='lidar_loc',
        name='lidar_loc',
        output='screen',
        parameters=[{
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'laser_frame': 'laser_frame',
            'laser_topic': 'scan',
            'update_freq': 30,
        }]
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    return LaunchDescription([
        map_server_node,
        jie_ware_node,
        lifecycle_manager,
    ])
