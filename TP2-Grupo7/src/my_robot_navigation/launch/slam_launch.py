import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_navigation = get_package_share_directory('my_robot_navigation')
    pkg_description = get_package_share_directory('my_robot_description')

    world_arg = DeclareLaunchArgument(
        'world', default_value='maze.world',
        description='Archivo de mundo dentro de my_robot_description/worlds/',
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'publish_diffdrive_tf': 'false',
        }.items(),
        )

    ground_truth_tf_node = Node(
        package='my_robot_navigation',
        executable='ground_truth_odom_tf',
        name='ground_truth_odom_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
        )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_navigation, 'config', 'slam_toolbox.yaml'),
            ],
        )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            }],
        )

    return LaunchDescription([
        world_arg,
        gazebo_launch,
        ground_truth_tf_node,
        slam_toolbox_node,
        lifecycle_manager_node,
        ])
