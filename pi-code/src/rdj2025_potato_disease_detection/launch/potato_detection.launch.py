from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rdj2025_potato_disease_detection',
            executable='publish_test_image',
            name='image_pub',
            output='screen',
            # parameters=[{'IP': '10.122.180.67'}] #b - Fundi
            # parameters=[{'IP': '10.226.56.67'}] #Kabbage - Gareth
            parameters=[{'IP': '192.168.0.112'}]# Gamefield
        ),
        Node(
            package='rdj2025_potato_disease_detection',
            executable='potato_disease_detection_node',
            name='disease_detector',
            output='screen'
        )
    ])
