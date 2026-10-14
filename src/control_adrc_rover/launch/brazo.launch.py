"""Lanza el control del brazo y, opcionalmente, el simulador de motores y
la interfaz web, con UN solo comando.

Uso:
    # Simulación (con Gazebo ya abierto): control + actuador_sim + interfaz,
    # y abre el navegador de Windows/Linux en http://localhost:8080
    ros2 launch control_adrc_rover brazo.launch.py

    # Robot real (Jetson): control + interfaz; abre en la PC del rover
    # http://<IP-de-la-Jetson>:8080 (la dirección sale en la terminal)
    ros2 launch control_adrc_rover brazo.launch.py sim:=false

    # Sólo J1 montada (J2..J5 ausentes)
    ros2 launch control_adrc_rover brazo.launch.py sim:=false montadas:=1

    # Estados a mano
    ros2 launch control_adrc_rover brazo.launch.py \
        estados:="Joint_4=bloqueada,Joint_5=ausente"

    # Sin interfaz (sólo el control, como antes)
    ros2 launch control_adrc_rover brazo.launch.py gui:=false

Argumentos:
    brazo        perfil de config/brazos/<brazo>.yaml (o ruta a un YAML)
    sim          true: plataforma "sim" + actuador_sim; false: "real"
    plataforma   sobrescribe la plataforma (nombre o ruta a un YAML)
    montadas     N > 0: monta las N primeras articulaciones; 0 = perfil
    estados      "Nombre=activa|bloqueada|ausente,..."
    gui          lanza la interfaz web (true por defecto)
    abrir_navegador  abre el navegador (por defecto, true en sim)
    puerto       puerto de la interfaz web (8080)

Autor: Adán Medina Covarrubias
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodos(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    sim = lc('sim').lower() == 'true'
    plataforma = lc('plataforma') or ('sim' if sim else 'real')
    abrir = lc('abrir_navegador')
    abrir = (sim if abrir == '' else abrir.lower() == 'true')
    comunes = {'brazo': lc('brazo'), 'plataforma': plataforma,
               'montadas': int(lc('montadas')), 'estados': lc('estados')}
    nodos = [Node(package='control_adrc_rover', executable='adrc_simple',
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
        ('sim', 'true', 'true: Gazebo + actuador_sim; false: robot real'),
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
