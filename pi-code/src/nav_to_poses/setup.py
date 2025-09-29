from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_to_poses'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/nav_goals.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='b',
    maintainer_email='me@gme.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'nav_to_poses = nav_to_poses.nav_to_poses:main'
        ],
    },
)
