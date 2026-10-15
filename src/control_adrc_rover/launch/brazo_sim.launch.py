"""Compatibilidad: el comando de siempre sigue funcionando.

    ros2 launch control_adrc_rover brazo_sim.launch.py            # simulación
    ros2 launch control_adrc_rover brazo_sim.launch.py sim:=false # robot real

Equivale a brazo.launch.py SIN interfaz web (como antes). Para la
interfaz usa brazo.launch.py (o gui:=true aquí). El viejo argumento
params_file ya no aplica: la configuración ahora está en
config/brazos/<brazo>.yaml + config/plataformas/<sim|real>.yaml.

Autor: Adán Medina Covarrubias
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _incluir(context, *args, **kwargs):
    acciones = []
    if LaunchConfiguration('params_file').perform(context):
        acciones.append(LogInfo(msg=(
            '[brazo_sim] params_file ya no se usa: la configuración está en '
            'config/brazos/ y config/plataformas/. Se ignora.')))
    base = os.path.join(get_package_share_directory('control_adrc_rover'),
                        'launch', 'brazo.launch.py')
    acciones.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base),
        launch_arguments={
            'sim': LaunchConfiguration('sim').perform(context),
            'gui': LaunchConfiguration('gui').perform(context),
            'brazo': LaunchConfiguration('brazo').perform(context),
            'montadas': LaunchConfiguration('montadas').perform(context),
        }.items()))
    return acciones


def generate_launch_description() -> LaunchDescription:
    """Argumentos heredados + los nuevos más usados."""
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('brazo', default_value='principal'),
        DeclareLaunchArgument('montadas', default_value='0'),
        OpaqueFunction(function=_incluir),
    ])
