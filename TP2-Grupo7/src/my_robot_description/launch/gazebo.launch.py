import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_description = get_package_share_directory('my_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- Argumentos configurables ---
    world_arg = DeclareLaunchArgument(
        'world', default_value='maze.world',
        description='Archivo de mundo dentro de la carpeta worlds/',
    )
    x_arg = DeclareLaunchArgument(
        'x', default_value='-2.025',
        description='Posicion inicial X del robot',
    )
    y_arg = DeclareLaunchArgument(
        'y', default_value='3.150',
        description='Posicion inicial Y del robot',
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Orientacion inicial (yaw) del robot en radianes',
    )

    world = LaunchConfiguration('world')

    # --- Resource path para Gazebo (encuentra las mallas STL) ---
    gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_description, '..'),
    )

    # --- Modelo del robot ---
    xacro_file = os.path.join(pkg_description, 'urdf', 'my_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    # --- Ruta del mundo (configurable por el argumento 'world') ---
    world_path = PathJoinSubstitution([pkg_description, 'worlds', world])

    # --- robot_state_publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_path]}.items(),
    )

    # --- Spawn del robot en la posicion de inicio ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.1',
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # --- Bridge ROS <-> Gazebo (todos los topics via archivo) ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_description, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        yaw_arg,
        gz_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
    ])