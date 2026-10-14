"""Núcleo del control del brazo, independiente de ROS.

Todo lo que está en este subpaquete se puede importar y probar sin ROS 2
(sólo numpy, PyYAML y, para el modelo dinámico, Pinocchio). Los nodos de
ROS (control_node, actuador_sim_node, gui) son capas delgadas encima.
"""
