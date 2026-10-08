"""Lanza el control del brazo con sus parámetros (incluida la interfaz ROS).

Uso:
    # Simulación (Gazebo ya abierto): usa config/brazo_adrc.yaml
    ros2 launch control_adrc_rover brazo_sim.launch.py
    # Robot real (Jetson): usa config/brazo_real.yaml, sin actuador_sim
    ros2 launch control_adrc_rover brazo_sim.launch.py sim:=false
    # Cualquier otro archivo
    ros2 launch control_adrc_rover brazo_sim.launch.py params_file:=/ruta/x.yaml

Los nombres de tópicos y articulaciones viven en el YAML (sección INTERFAZ);
este archivo no necesita cambios al pasar a la Jetson.

Autor: Adán Medina Covarrubias
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodos(context, *args, **kwargs):
    """Elige el YAML según `sim` (si no se dio params_file) y crea nodos."""
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(
            get_package_share_directory('control_adrc_rover'), 'config',
            'brazo_adrc.yaml' if sim else 'brazo_real.yaml')

    nodos = [Node(
        package='control_adrc_rover',
        executable='adrc_simple',
        name='adrc_simple',
        output='screen',
        parameters=[params_file])]
    if sim:
        nodos.append(Node(
            package='control_adrc_rover',
            executable='actuador_sim',
            name='actuador_sim',
            output='screen',
            parameters=[params_file]))
    return nodos


def generate_launch_description() -> LaunchDescription:
    """Declara argumentos y nodos."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='true: Gazebo + actuador_sim; false: robot real'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='YAML de parámetros (vacío = según `sim`)'),
        OpaqueFunction(function=_nodos),
    ])
