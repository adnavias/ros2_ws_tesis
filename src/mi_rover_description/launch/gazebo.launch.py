"""Abre Gazebo Fortress con el rover y el brazo, y el puente ROS <-> Gazebo.

Uso:
    ros2 launch mi_rover_description gazebo.launch.py
    ros2 launch mi_rover_description gazebo.launch.py montadas:=1
    ros2 launch mi_rover_description gazebo.launch.py escala_masa:=1.3

Argumentos:
    montadas     articulaciones del brazo montadas (1..5; por defecto 5).
                 Con montadas:=1 sólo existe la base giratoria (J1), como
                 en las pruebas reales con el brazo parcialmente armado.
                 Debe coincidir con el `montadas` del control.
    escala_masa  factor de masa e inercia de los eslabones del brazo para
                 probar la robustez del control (el control no lo sabe).
    puente       true (por defecto): lanza ros_gz_bridge con la posición
                 de las articulaciones, los torques, el reloj, /cmd_vel y
                 la IMU (/imu_data). Pon
                 puente:=false si ya corres tu propio parameter_bridge.

Autor: Adán Medina Covarrubias
"""

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

MODELO = 'mi_rover'
MUNDO = 'empty'


def _nodos(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mi_rover_description')
    montadas = int(LaunchConfiguration('montadas').perform(context))
    escala = LaunchConfiguration('escala_masa').perform(context)
    puente = LaunchConfiguration('puente').perform(context).lower() == 'true'
    if not 1 <= montadas <= 5:
        raise RuntimeError('montadas debe estar entre 1 y 5')

    xacro_file = os.path.join(pkg_share, 'urdf', 'rover_completo.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={
        'montadas': str(montadas), 'escala_masa': escala})
    robot_description = {'robot_description': doc.toxml()}

    nodos = [
        Node(package='robot_state_publisher',
             executable='robot_state_publisher', output='screen',
             parameters=[robot_description]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', MODELO,
                        '-z', '0.0']),
    ]
    if puente:
        puentes = [
            f'/world/{MUNDO}/model/{MODELO}/joint_state'
            '@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/imu_data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ]
        for k in range(1, montadas + 1):
            puentes.append(f'/model/{MODELO}/joint/Joint_{k}/cmd_force'
                           '@std_msgs/msg/Float64]ignition.msgs.Double')
        nodos.append(Node(package='ros_gz_bridge',
                          executable='parameter_bridge', output='screen',
                          arguments=puentes, name='puente_brazo'))
    return nodos


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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items())

    return LaunchDescription(env_actions + [
        DeclareLaunchArgument('montadas', default_value='5',
                              description='Articulaciones montadas (1..5)'),
        DeclareLaunchArgument('escala_masa', default_value='1.0',
                              description='Escala de masa del brazo'),
        DeclareLaunchArgument('puente', default_value='true',
                              description='Lanzar ros_gz_bridge'),
        gazebo,
        OpaqueFunction(function=_nodos),
    ])
