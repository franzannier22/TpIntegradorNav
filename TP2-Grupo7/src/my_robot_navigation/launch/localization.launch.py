import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Directorios instalados de nuestros paquetes
    pkg_navigation = get_package_share_directory(
        'my_robot_navigation'
    )

    pkg_description = get_package_share_directory(
        'my_robot_description'
    )

    # Archivos utilizados para localización
    default_map_file = os.path.join(
        pkg_navigation,
        'maps',
        'living_room.yaml'
    )

    amcl_params_file = os.path.join(
        pkg_navigation,
        'config',
        'amcl.yaml'
    )

    # Argumentos configurables
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='living_room.world',
        description='Mundo de Gazebo correspondiente al mapa'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Archivo YAML del mapa conocido'
    )

    x_arg = DeclareLaunchArgument(
        'x',
        description='Posición inicial X del robot en Gazebo'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        description='Posición inicial Y del robot en Gazebo'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Orientación inicial del robot en radianes'
    )

    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    # Simulación, robot y bridge ROS <-> Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_description,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world,
            'x': x,
            'y': y,
            'yaw': yaw,
            'publish_diffdrive_tf': 'true',
        }.items()
    )

    # Carga y publica el mapa guardado
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'yaml_filename': map_file,
            }
        ]
    )

    # Localización del robot sobre el mapa
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_params_file,
            {
                'use_sim_time': True,
            }
        ]
    )

    # Configura y activa map_server y AMCL
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                ],
            }
        ]
    )

    # Visualización
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
            }
        ]
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        x_arg,
        y_arg,
        yaw_arg,

        gazebo_launch,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node,
    ])