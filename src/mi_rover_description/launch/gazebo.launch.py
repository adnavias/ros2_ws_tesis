import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = 'mi_rover_description'
    pkg_share = get_package_share_directory(pkg_name)
    pkg_prefix = get_package_prefix(pkg_name)
    lib_path = os.path.join(pkg_prefix, 'lib')
    mesh_path = os.path.join(pkg_share, '..')

    # Rutas de mallas y plugins (Fortress usa IGN_*, se dejan GZ_* por compatibilidad)
    env_actions = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', mesh_path),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', mesh_path),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', lib_path),
        AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', lib_path),
    ]

    # Procesar el xacro maestro
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover_completo.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Publicador del estado del robot
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description])

    # Gazebo Fortress
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items())

    # Spawn del robot (la postura inicial la pone el plugin)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'mi_rover',
                   '-z', '0.0'],
        output='screen')

    return LaunchDescription(env_actions + [
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
