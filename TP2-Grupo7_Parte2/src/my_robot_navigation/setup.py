from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanma-painenao',
    maintainer_email='painenaojuanmanuel@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follower = my_robot_navigation.wall_follower:main',
            'occupancy_grid = my_robot_navigation.occupancy_grid:main',
            'camera = my_robot_navigation.camera:main',
            'ground_truth_odom_tf = my_robot_navigation.ground_truth_odom_tf:main',
        ],
    },
)
