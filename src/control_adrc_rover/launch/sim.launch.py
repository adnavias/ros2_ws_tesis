"""SIMULACIÓN (Gazebo ya abierto): control + actuador_sim + interfaz.

    ros2 launch control_adrc_rover sim.launch.py
    ros2 launch control_adrc_rover sim.launch.py montadas:=1

Usa config/plataformas/sim.yaml y guarda/carga las ganancias en
config/ajustes/<brazo>_sim.yaml. Acepta los mismos argumentos que
brazo.launch.py (brazo, montadas, estados, gui, abrir_navegador, puerto).

Autor: Adán Medina Covarrubias
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ARGS = [('brazo', 'principal'), ('montadas', '0'), ('estados', ''),
        ('archivo_ajustes', ''), ('gui', 'true'), ('abrir_navegador', ''),
        ('puerto', '8080')]


def generate_launch_description() -> LaunchDescription:
    """Incluye brazo.launch.py con sim:=true."""
    base = os.path.join(get_package_share_directory('control_adrc_rover'),
                        'launch', 'brazo.launch.py')
    argumentos = {n: LaunchConfiguration(n) for n, _ in ARGS}
    argumentos['sim'] = 'true'
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=v) for n, v in ARGS]
        + [IncludeLaunchDescription(PythonLaunchDescriptionSource(base),
                                    launch_arguments=argumentos.items())])
