"""Modelo de actuadores del brazo para simulación (nodo "actuador_sim").

Gazebo no simula corriente: este nodo hace el papel físico de cada
actuador. Lee el MISMO perfil y la misma plataforma que el control (así
sabe qué articulaciones hay, qué motor tiene cada una y en qué tópicos
llega el mando) y publica el torque en Gazebo (ApplyJointForce):

    corriente_con_modelo   tau = K_tau sign(i) max(|i| - i0, 0)
                           K_tau = k_t_caja * relacion_sinfin * eficiencia_sinfin
    torque_sin_modelo      tau = mando
    posicion_directa       servo simulado: PD sobre la posición medida
                           tau = kp (q_cmd - q) - kd q' (saturado)
    bloqueada / ausente    si la articulación existe en Gazebo, se
                           sostiene con un PD rígido en su postura inicial
                           (`sostener_no_activas`). Para simular de verdad
                           un brazo con menos articulaciones, lanza Gazebo
                           con montadas:=N (el xacro las quita).

Los parámetros del motor son los de `motores` del perfil, y se pueden
cambiar SÓLO para la planta simulada con la sección `actuador_sim.motores`
de la plataforma (p. ej. eficiencia_sinfin: 0.4) para probar la robustez
ante un K_tau mal estimado. En el robot real este nodo NO se usa.

Seguridad: si deja de llegar mando de corriente por `timeout_mando`
segundos, ese torque se pone en cero (como un driver con watchdog).

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from control_adrc_rover.nucleo import config as cfgmod
from control_adrc_rover.rutas import ruta_capa

DEFAULTS_SIM = {
    'motores': {},
    'servo_kp': 20.0,
    'servo_kd': 1.0,
    'servo_torque_max': 2.0,
    'sostener_no_activas': True,
    'sostener_kp': 300.0,
    'sostener_kd': 10.0,
    'sostener_torque_max': 60.0,
    'timeout_mando': 0.1,
    'frecuencia': 200.0,
}


def corriente_a_torque(i: float, motor: Optional[dict]) -> float:
    """Corriente [A] -> torque articular [N·m] con zona muerta i0."""
    if motor is None:
        return i
    k_tau = (float(motor['k_t_caja']) * float(motor.get('relacion_sinfin', 1.0))
             * float(motor.get('eficiencia_sinfin', 1.0)))
    i_max = float(motor.get('corriente_bloqueo', 1e9))
    i = max(-i_max, min(i_max, i))
    util = max(abs(i) - float(motor.get('corriente_vacio', 0.0)), 0.0)
    return k_tau * util if i >= 0.0 else -k_tau * util


class _Junta:
    """Estado por articulación simulada."""

    def __init__(self, art: cfgmod.Articulacion, modo: str,
                 motor: Optional[dict], pub) -> None:
        self.art = art
        self.modo = modo          # corriente | torque | servo | sostener
        self.motor = motor
        self.pub = pub
        self.t_mando: Optional[float] = None
        self.q: Optional[float] = None
        self.qd = 0.0
        self.t_q: Optional[float] = None
        self.objetivo: Optional[float] = None


class ActuadorSim(Node):
    """Traduce el mando del controlador a torque para Gazebo."""

    def __init__(self) -> None:
        """Lee la configuración y crea suscripciones y publicadores."""
        super().__init__('actuador_sim')
        for nombre, valor in (('brazo', 'principal'), ('plataforma', 'sim'),
                              ('montadas', 0), ('estados', '')):
            self.declare_parameter(nombre, valor)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        cfg = cfgmod.cargar(ruta_capa(str(gp('brazo')), 'brazos'),
                            ruta_capa(str(gp('plataforma')), 'plataformas'),
                            '', int(gp('montadas')),
                            cfgmod.parsear_estados(str(gp('estados'))))
        self.cfg = cfg
        sim = cfgmod.fusionar(DEFAULTS_SIM, cfg.crudo.get('actuador_sim') or {})
        self.sim = sim
        motores = cfgmod.fusionar(cfg.crudo.get('motores') or {},
                                  sim['motores'] or {})
        itf = cfg.interfaz
        self.juntas: Dict[str, _Junta] = {}
        for a in cfg.articulaciones:
            topico = itf['patron_topico_fuerza'].format(articulacion=a.nombre)
            if a.estado == 'activa':
                modo = {'corriente_con_modelo': 'corriente',
                        'torque_sin_modelo': 'torque',
                        'posicion_directa': 'servo'}[a.mando]
            elif sim['sostener_no_activas']:
                modo = 'sostener'
            else:
                continue
            motor = motores.get(a.motor) if modo == 'corriente' else None
            pub = self.create_publisher(Float64, topico, 10)
            self.juntas[a.nombre] = _Junta(a, modo, motor, pub)

        # Mando (misma interfaz que el control)
        tipo = itf['tipo_mando']
        art_crudo = cfg.crudo.get('articulaciones', {})
        self.por_nombre_mando = {j.art.nombre_en_mando: j
                                 for j in self.juntas.values()}
        if tipo == 'float64_por_articulacion':
            for j in self.juntas.values():
                if j.art.estado != 'activa':
                    continue
                topico = (art_crudo.get(j.art.nombre, {}) or {}).get(
                    'topico_mando') or itf['patron_topico_mando'].format(
                        articulacion=j.art.nombre_en_mando)
                self.create_subscription(
                    Float64, topico,
                    lambda msg, j=j: self._mando(j, msg.data), 10)
        elif tipo == 'float64_multiarray':
            self.activas = [self.juntas[a.nombre] for a in cfg.movibles]
            self.create_subscription(Float64MultiArray,
                                     itf['topico_mando_unico'],
                                     self._cb_multiarray, 10)
        else:
            self.create_subscription(JointState, itf['topico_mando_unico'],
                                     self._cb_joint_state, 10)
        qos = (qos_profile_sensor_data if itf['qos_estado_best_effort']
               else QoSProfile(depth=10))
        self.create_subscription(JointState, itf['topico_estado'],
                                 self._cb_estado, qos)
        for j in self.juntas.values():
            if j.modo == 'corriente':
                m = j.motor
                k = (float(m['k_t_caja']) * float(m.get('relacion_sinfin', 1))
                     * float(m.get('eficiencia_sinfin', 1)))
                self.get_logger().info(
                    f'{j.art.nombre}: corriente, K_tau = {k:.3f} N·m/A, '
                    f'i0 = {m.get("corriente_vacio", 0.0)} A')
            else:
                self.get_logger().info(f'{j.art.nombre}: {j.modo}')
        self.create_timer(1.0 / float(sim['frecuencia']), self._cb_timer)

    # ------------------------------------------------------------------
    def _mando(self, j: _Junta, valor: float) -> None:
        if valor is None or not math.isfinite(valor):
            return
        j.t_mando = time.monotonic()
        if j.modo == 'servo':
            crudo = float(valor) if j.art.unidad_posicion == 'rad' \
                else math.radians(float(valor))
            # el mando del servo viene en la convención del driver:
            # q_urdf = signo * crudo + offset (igual que la medición)
            offset = (j.art.postura_encendido_urdf
                      if j.art.encoder == 'incremental'
                      else j.art.offset_absoluto)
            j.objetivo = j.art.signo * crudo + offset
            return
        if j.modo == 'corriente':
            tau = corriente_a_torque(j.art.signo * valor, j.motor)
        elif j.modo == 'torque':
            tau = j.art.signo * valor
        else:
            return
        j.pub.publish(Float64(data=float(tau)))

    def _cb_multiarray(self, msg: Float64MultiArray) -> None:
        for j, valor in zip(self.activas, msg.data):
            self._mando(j, valor)

    def _cb_joint_state(self, msg: JointState) -> None:
        for k, nombre in enumerate(msg.name):
            j = self.por_nombre_mando.get(nombre)
            if j is None:
                continue
            if j.modo == 'servo' and k < len(msg.position):
                self._mando(j, msg.position[k])
            elif k < len(msg.effort):
                self._mando(j, msg.effort[k])

    def _cb_estado(self, msg: JointState) -> None:
        st = msg.header.stamp
        t = st.sec + st.nanosec * 1e-9
        for nombre, q in zip(msg.name, msg.position):
            j = self.juntas.get(nombre)
            if j is None:
                continue
            if j.q is not None and j.t_q is not None and t > j.t_q:
                j.qd = 0.7 * j.qd + 0.3 * (q - j.q) / (t - j.t_q)
            j.q, j.t_q = q, t
            if j.objetivo is None and j.modo in ('servo', 'sostener'):
                j.objetivo = q            # sostiene donde arrancó

    def _cb_timer(self) -> None:
        ahora = time.monotonic()
        sim = self.sim
        for j in self.juntas.values():
            if j.modo == 'corriente' or j.modo == 'torque':
                if j.t_mando is not None and (
                        ahora - j.t_mando > float(sim['timeout_mando'])):
                    j.pub.publish(Float64(data=0.0))
                    j.t_mando = None
                    self.get_logger().warning(
                        f'{j.art.nombre}: sin mando por '
                        f'{sim["timeout_mando"]} s -> torque 0')
                continue
            if j.q is None or j.objetivo is None:
                continue
            if j.modo == 'servo':
                kp, kd, tmax = (float(sim['servo_kp']), float(sim['servo_kd']),
                                float(sim['servo_torque_max']))
            else:
                kp, kd, tmax = (float(sim['sostener_kp']),
                                float(sim['sostener_kd']),
                                float(sim['sostener_torque_max']))
            tau = kp * (j.objetivo - j.q) - kd * j.qd
            j.pub.publish(Float64(data=float(max(-tmax, min(tmax, tau)))))


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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
