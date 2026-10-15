"""Lanza el control del brazo y, opcionalmente, el simulador de motores y
la interfaz web, con UN solo comando.

Uso:
    # Lo más corto: detecta solo dónde corre. En la Jetson usa la
    # plataforma "real"; en la PC (WSL), "sim".
    ros2 launch control_adrc_rover brazo.launch.py

    # Explícito (no depende de la detección):
    ros2 launch control_adrc_rover sim.launch.py     # Gazebo + actuador_sim
    ros2 launch control_adrc_rover real.launch.py    # robot real

    # Robot real: abre en la PC del rover http://<IP-de-la-Jetson>:8080
    # (la dirección sale en la terminal)

    # Forzar con una variable de entorno (p. ej. en ~/.bashrc de la Jetson):
    export BRAZO_PLATAFORMA=real

    # Sólo J1 montada (J2..J5 ausentes)
    ros2 launch control_adrc_rover brazo.launch.py sim:=false montadas:=1

    # Estados a mano
    ros2 launch control_adrc_rover brazo.launch.py \
        estados:="Joint_4=bloqueada,Joint_5=ausente"

    # Sin interfaz (sólo el control, como antes)
    ros2 launch control_adrc_rover brazo.launch.py gui:=false

Argumentos:
    brazo        perfil de config/brazos/<brazo>.yaml (o ruta a un YAML)
    sim          true: plataforma "sim" + actuador_sim; false: "real";
                 vacío (por defecto): BRAZO_PLATAFORMA o detección (Jetson
                 -> real, si no -> sim)
    plataforma   sobrescribe la plataforma (nombre o ruta a un YAML)
    montadas     N > 0: monta las N primeras articulaciones; 0 = perfil
    estados      "Nombre=activa|bloqueada|ausente,..."
    gui          lanza la interfaz web (true por defecto)
    abrir_navegador  abre el navegador (por defecto, true en sim)
    puerto       puerto de la interfaz web (8080)

Autor: Adán Medina Covarrubias
"""

import os
import platform as _plataforma

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def es_jetson() -> bool:
    """True si corre en una NVIDIA Jetson (L4T)."""
    if os.path.exists('/etc/nv_tegra_release'):
        return True
    try:
        with open('/proc/device-tree/model', 'rb') as fh:
            if b'jetson' in fh.read().lower():
                return True
    except OSError:
        pass
    return 'tegra' in _plataforma.release().lower()


def elegir_sim(arg_sim: str, arg_plataforma: str):
    """(sim, motivo) a partir de los argumentos, el entorno y el equipo."""
    if arg_sim:
        return arg_sim.lower() == 'true', f'argumento sim:={arg_sim}'
    if arg_plataforma:
        return arg_plataforma == 'sim', f'argumento plataforma:={arg_plataforma}'
    env = os.environ.get('BRAZO_PLATAFORMA', '').strip().lower()
    if env in ('sim', 'real'):
        return env == 'sim', f'BRAZO_PLATAFORMA={env}'
    if es_jetson():
        return False, 'se detectó una Jetson'
    return True, 'no es una Jetson'


def _nodos(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    sim, motivo = elegir_sim(lc('sim'), lc('plataforma'))
    plataforma = lc('plataforma') or ('sim' if sim else 'real')
    abrir = lc('abrir_navegador')
    abrir = (sim if abrir == '' else abrir.lower() == 'true')
    comunes = {'brazo': lc('brazo'), 'plataforma': plataforma,
               'montadas': int(lc('montadas')), 'estados': lc('estados')}
    nodos = [LogInfo(msg=f'[brazo] Plataforma "{plataforma}" ({motivo}); '
                         f'ajustes en config/ajustes/{lc("brazo")}_'
                         f'{plataforma}.yaml'),
             Node(package='control_adrc_rover', executable='adrc_simple',
                  name='adrc_simple', output='screen',
                  parameters=[dict(comunes, autoarranque=True,
                                   archivo_ajustes=lc('archivo_ajustes'))])]
    if sim:
        nodos.append(Node(package='control_adrc_rover',
                          executable='actuador_sim', name='actuador_sim',
                          output='screen', parameters=[comunes]))
    if lc('gui').lower() == 'true':
        nodos.append(Node(package='control_adrc_gui', executable='control_gui',
                          name='control_gui', output='screen',
                          parameters=[{'puerto': int(lc('puerto')),
                                       'abrir_navegador': abrir}]))
    return nodos


def generate_launch_description() -> LaunchDescription:
    """Declara argumentos y nodos."""
    args = [
        ('brazo', 'principal', 'Perfil de brazo (config/brazos/<brazo>.yaml)'),
        ('sim', '', 'true: Gazebo + actuador_sim; false: robot real; '
                    'vacío: detección automática'),
        ('plataforma', '', 'Plataforma (vacío = sim o real según `sim`)'),
        ('montadas', '0', 'Articulaciones montadas (0 = según el perfil)'),
        ('estados', '', 'Estados: "Joint_2=ausente,Joint_3=bloqueada"'),
        ('archivo_ajustes', '', 'YAML de ajustes (vacío = autotune_<brazo>)'),
        ('gui', 'true', 'Lanzar la interfaz web'),
        ('abrir_navegador', '', 'Abrir el navegador (vacío = sí en sim)'),
        ('puerto', '8080', 'Puerto de la interfaz web'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=v, description=d)
         for n, v, d in args] + [OpaqueFunction(function=_nodos)])
