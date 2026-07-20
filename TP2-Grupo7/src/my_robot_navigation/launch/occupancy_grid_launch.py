import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('my_robot_description'),
        'rviz',
        'urdf_config.rviz'
    )

    params_file = os.path.join(
        get_package_share_directory('my_robot_navigation'),
        'config',
        'occupancy_grid.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    occupancy_grid_node = Node(
        package='my_robot_navigation',
        executable='occupancy_grid',
        name='occupancy_grid',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        occupancy_grid_node,
    ])