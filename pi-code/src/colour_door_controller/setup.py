from setuptools import setup

package_name = 'colour_door_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='d',
    maintainer_email='dismaskarimidissy64@gmail.com',
    description='ROS2 package for controlling a door using colour detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'colour_to_door_service = colour_door_controller.colour_to_door_service:main',
            'colour_publisher = colour_door_controller.colour_publisher:main',
        ],
    },
)
