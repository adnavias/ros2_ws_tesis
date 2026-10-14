cmake_minimum_required(VERSION 3.8)
project(control_adrc_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EstadoControl.msg"
  "srv/CargarPerfil.srv"
  "srv/GuardarConfiguracion.srv"
  "action/MoverA.action"
  "action/Autotune.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
