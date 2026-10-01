import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('mi_rover_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'rover_completo.urdf.xacro')
    # Le decimos dónde está tu configuración guardada
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # Le pasamos el argumento "-d" para que cargue tu archivo .rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
