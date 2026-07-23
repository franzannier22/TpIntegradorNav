import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Se ejecuta en tiempo de lanzamiento: aca ya podemos leer los valores
    de los argumentos (x, y, yaw) y convertirlos a float para la pose inicial
    de AMCL."""

    pkg_navigation = get_package_share_directory('my_robot_navigation')
    pkg_description = get_package_share_directory('my_robot_description')

    amcl_params_file = os.path.join(pkg_navigation, 'config', 'amcl.yaml')
    nav2_params_file = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_description, 'rviz', 'urdf_config.rviz')

    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    # Valores concretos (string -> float) para la pose inicial de AMCL
    x_val = float(context.perform_substitution(x))
    y_val = float(context.perform_substitution(y))
    yaw_val = float(context.perform_substitution(yaw))

    # Simulacion, robot y bridge ROS <-> Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'x': x,
            'y': y,
            'yaw': yaw,
            'publish_diffdrive_tf': 'true',
        }.items()
    )

    # Carga y publica el mapa guardado (capa estatica del costmap)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_file}],
    )

    # Localizacion (provee TF map -> odom). Le damos una pose inicial automatica
    # para que la TF exista desde el arranque y los costmaps puedan activarse.
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_params_file,
            {
                'use_sim_time': True,
                'set_initial_pose': True,
                'initial_pose.x': x_val,
                'initial_pose.y': y_val,
                'initial_pose.z': 0.0,
                'initial_pose.yaw': yaw_val,
            },
        ],
    )

    # Path planner global (A*/Dijkstra sobre el global costmap)
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # Motion planner (DWB / RPP) con global y local costmaps
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # Gestiona el ciclo de vida de todos los nodos Nav2
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
            ],
        }],
    )

    # Visualizacion
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return [
        gazebo_launch,
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        lifecycle_manager_node,
        rviz_node,
    ]


def generate_launch_description():

    pkg_navigation = get_package_share_directory('my_robot_navigation')
    default_map_file = os.path.join(pkg_navigation, 'maps', 'living_room.yaml')

    # --- Argumentos configurables (defaults para living_room) ---
    world_arg = DeclareLaunchArgument(
        'world', default_value='living_room.world',
        description='Mundo de Gazebo correspondiente al mapa',
    )
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_file,
        description='Archivo YAML del mapa conocido',
    )
    x_arg = DeclareLaunchArgument(
        'x', default_value='1.8',
        description='Posicion inicial X del robot (spawn y pose inicial de AMCL)',
    )
    y_arg = DeclareLaunchArgument(
        'y', default_value='-1.9',
        description='Posicion inicial Y del robot (spawn y pose inicial de AMCL)',
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Orientacion inicial del robot en radianes',
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        x_arg,
        y_arg,
        yaw_arg,
        OpaqueFunction(function=launch_setup),
    ])
