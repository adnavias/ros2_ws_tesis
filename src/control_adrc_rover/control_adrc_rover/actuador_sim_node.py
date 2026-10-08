"""Modelo de actuadores del brazo para simulación (corriente -> torque).

Gazebo no simula corriente: este nodo hace el papel físico de
"motor + caja + sinfín". Recibe el mando del controlador (mismos
parámetros de interfaz que adrc_simple: sección /** del YAML) y publica el
torque de cada articulación en `topicos_fuerza` (ApplyJointForce de Gazebo).

    J1-J4 (corriente):  tau = K_tau * sign(i) * max(|i| - i0, 0)
                        con K_tau = K_t * N_caja * eta_caja * N_sinfin * eta_sinfin
                        e i0 la corriente en vacío (fricción interna).
    J5 (servo, torque): tau = cmd

Estos son los parámetros "reales" de la planta simulada. Son
independientes de K_TAU_HAT del controlador: puedes cambiarlos aquí (p. ej.
eta_sinfin = 0.4) para probar la robustez del ADRC ante un K_tau mal
estimado. En el robot real este nodo NO se usa.

Seguridad: si deja de llegar mando durante CMD_TIMEOUT, el torque se pone
en cero (como haría un driver con watchdog).

Autor: Adán Medina Covarrubias
"""

from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

# =====================================================================
# PARÁMETROS FÍSICOS DE LOS ACTUADORES (editar aquí)
# =====================================================================
# Yellow Jacket 5203, 50.9:1 a 12 V: par de bloqueo 6.71 N·m, corriente de
# bloqueo 9.2 A, en vacío 0.25 A -> K_t * N * eta = 0.75 N·m/A en la salida.
YELLOW_JACKET = dict(
    kt_caja=6.71 / (9.2 - 0.25),  # [N·m/A] ya incluye N_caja y eta_caja
    n_sinfin=10.0,                # relación del tornillo sin fin (supuesta)
    eta_sinfin=0.5,               # eficiencia del sinfín (supuesta)
    i0=0.25,                      # [A] corriente en vacío
    i_bloqueo=9.2,                # [A] corriente máx. física (V/R)
)

# Maxon EC 60 flat 642221 (24 V) + reductor strain wave GSW 62 A 867073.
MAXON = dict(
    kt_caja=0.0525 * 100.0 * 0.75,  # Kt 52.5 mN·m/A, 100:1, eta_max 75 %
    n_sinfin=1.0,
    eta_sinfin=1.0,
    i0=0.497,                       # [A] corriente en vacío
    i_bloqueo=83.2,                 # [A] corriente de arranque
)

# Asignación por articulación (None = mando directo en torque).
ACTUADORES: Dict[str, Optional[dict]] = {
    'Joint_1': YELLOW_JACKET,
    'Joint_2': MAXON,
    'Joint_3': MAXON,
    'Joint_4': YELLOW_JACKET,
    'Joint_5': None,  # servo de rotación continua
}

# Los tópicos se leen de los parámetros (sección /** del YAML), para usar
# exactamente la misma interfaz de mando que el controlador.
DEFAULT_NAMES = list(ACTUADORES.keys())
CMD_TIMEOUT: float = 0.1     # [s] sin mando -> torque cero
WATCHDOG_RATE: float = 20.0  # [Hz]
# =====================================================================


def corriente_a_torque(i: float, act: Optional[dict]) -> float:
    """Convierte corriente [A] en torque articular [N·m]."""
    if act is None:
        return i
    k_tau = act['kt_caja'] * act['n_sinfin'] * act['eta_sinfin']
    i = max(-act['i_bloqueo'], min(act['i_bloqueo'], i))
    i_util = max(abs(i) - act['i0'], 0.0)
    return k_tau * i_util if i >= 0.0 else -k_tau * i_util


class ActuadorSim(Node):
    """Traduce el mando del controlador a torque para Gazebo."""

    def __init__(self) -> None:
        """Lee la interfaz, crea suscripciones, publicadores y watchdog."""
        super().__init__('actuador_sim')
        self.declare_parameter('nombres_urdf', DEFAULT_NAMES)
        self.declare_parameter('tipo_mando', 'float64_por_articulacion')
        self.declare_parameter(
            'topicos_mando', [f'/brazo/{n}/cmd_actuador' for n in DEFAULT_NAMES])
        self.declare_parameter('topico_mando_unico', '/brazo/cmd_actuador')
        self.declare_parameter('nombres_en_mando', DEFAULT_NAMES)
        self.declare_parameter(
            'topicos_fuerza',
            [f'/model/mi_rover/joint/{n}/cmd_force' for n in DEFAULT_NAMES])
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.names = list(gp('nombres_urdf'))
        self.acts = [ACTUADORES.get(n) for n in self.names]
        tipo = gp('tipo_mando')

        self.force_pubs = [self.create_publisher(Float64, t, 10)
                           for t in gp('topicos_fuerza')]
        self.last_ns: List[Optional[int]] = [None] * len(self.names)

        if tipo == 'float64_por_articulacion':
            for k, topic in enumerate(gp('topicos_mando')):
                self.create_subscription(
                    Float64, topic,
                    lambda msg, k=k: self._aplicar(k, msg.data), 10)
            origen = list(gp('topicos_mando'))
        elif tipo == 'float64_multiarray':
            self.create_subscription(
                Float64MultiArray, gp('topico_mando_unico'),
                self._multiarray_cb, 10)
            origen = gp('topico_mando_unico')
        elif tipo == 'joint_state':
            self.nombres_mando = list(gp('nombres_en_mando'))
            self.create_subscription(
                JointState, gp('topico_mando_unico'), self._joint_state_cb, 10)
            origen = gp('topico_mando_unico')
        else:
            raise ValueError(f'tipo_mando="{tipo}" no válido')

        for name, act in zip(self.names, self.acts):
            if act is not None:
                k = act['kt_caja'] * act['n_sinfin'] * act['eta_sinfin']
                self.get_logger().info(
                    f'{name}: K_tau = {k:.3f} N·m/A, i0 = {act["i0"]} A')
            else:
                self.get_logger().info(f'{name}: mando directo en torque')
        self.get_logger().info(
            f'Mando ({tipo}) <- {origen} | torque -> {gp("topicos_fuerza")}')
        self.create_timer(1.0 / WATCHDOG_RATE, self._watchdog)

    def _aplicar(self, k: int, valor: float) -> None:
        """Publica de inmediato el torque de la articulación k."""
        self.last_ns[k] = self.get_clock().now().nanoseconds
        self.force_pubs[k].publish(
            Float64(data=float(corriente_a_torque(valor, self.acts[k]))))

    def _multiarray_cb(self, msg: Float64MultiArray) -> None:
        """Mando [J1..J5] en un solo arreglo."""
        for k, valor in enumerate(msg.data[:len(self.names)]):
            self._aplicar(k, valor)

    def _joint_state_cb(self, msg: JointState) -> None:
        """Mando en el campo effort de un JointState."""
        for name, valor in zip(msg.name, msg.effort):
            if name in self.nombres_mando:
                self._aplicar(self.nombres_mando.index(name), valor)

    def _watchdog(self) -> None:
        """Pone torque cero en las articulaciones sin mando reciente."""
        now = self.get_clock().now().nanoseconds
        for k, stamp in enumerate(self.last_ns):
            if stamp is not None and (now - stamp) * 1e-9 > CMD_TIMEOUT:
                self.force_pubs[k].publish(Float64(data=0.0))
                self.last_ns[k] = None
                self.get_logger().warning(
                    f'{self.names[k]}: sin mando por {CMD_TIMEOUT} s -> '
                    f'torque 0')


def main(args: Optional[List[str]] = None) -> None:
    """Punto de entrada del nodo."""
    rclpy.init(args=args)
    node = ActuadorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
