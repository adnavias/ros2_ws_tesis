"""ADRC por articulación con trayectoria quíntica, inversa de B por modelo
dinámico (Pinocchio) y salida en CORRIENTE.

Flujo dentro del mismo nodo y del mismo lazo:

    objetivo [°] + vel [%] --> trayectoria quíntica (q*, q*', q*'')
        --> ADRC (v) --> tau = M_hat(q) v + g_hat(q) --> i = tau / K_tau_hat
        --> saturación de corriente --> actuadores

Todos los parámetros se leen de config/brazo_adrc.yaml (ver ese archivo),
incluida la INTERFAZ ROS: tópicos, nombres de articulaciones en los
mensajes, sentido de giro, QoS y formato del mando. Para otra plataforma
(Jetson) sólo se cambia el YAML; este código no se toca.

    suscriptor objetivo [°]  ┐
    suscriptor estado  [rad] ┴─> trayectoria + ADRC + M(q) ─> publicador mando

ENTRADA, en grados y relativa a la postura cero; 6.º valor opcional = % de
velocidad (100 % = factor_100pct * velocidad física; tope 200 %):
    ros2 topic pub --once /brazo/objetivo_deg std_msgs/msg/Float64MultiArray \
        "{data: [0.0, 135.0, -135.0, 0.0, 0.0, 20.0]}"
Un NaN deja esa articulación en su objetivo actual.

Ley de control. Cada articulación es, nominalmente, un doble integrador
q_j'' = v_j. El compensador GPI (Sira-Ramírez) genera la entrada virtual

    e   = q - q*,
    s2' = -k3 s2 + e,   s1' = s2,
    v   = q*'' - (k2 s2' + k1 s2 + k0 s1),

con polinomio característico (s^2 + 2 zeta wn s + wn^2)(s^2 + 2 zeta wp s
+ wp^2), wp = wn/eps. La dinámica del brazo es M(q) q'' + C q' + g(q) =
K_tau i, es decir q'' = B(q) i + f con B(q) = M(q)^-1 K_tau. Se aplica la
inversa de B:

    i = K_tau_hat^-1 ( M_hat(q) v + g_hat(q) ) + i0 sgn(i),

donde M_hat depende de `modo_inercia`:
    constante -> diag(J_hat)               (b0 fijo, como antes)
    diagonal  -> diag(M(q))                (b0 programado por postura)
    completa  -> M(q)                      (desacoplamiento MIMO)
Coriolis, fricción y el error de modelo los absorbe el observador. El
término i0 sgn(i) compensa la corriente en vacío (fricción interna del
motor), suavizado con tanh para no conmutar bruscamente cerca de cero.

Trayectorias factibles en corriente: al recibir un objetivo, la curva se
revisa con el modelo, i = K^-1 (M(q*) q*'' + g(q*)) + i0, y si en algún punto
supera `margen_corriente_trayectoria` * límite, se alarga la duración hasta
que cabe. Así el brazo nunca pide más corriente de la permitida, aun
estirado (donde la gravedad consume casi todo el par disponible de J2).

Salida en /brazo/<Joint_i>/cmd_actuador (Float64): J1-J4 corriente [A],
J5 torque [N·m].

Autor: Adán Medina Covarrubias
"""

import os
from typing import List, Optional

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from control_adrc_rover.trayectoria import TrayectoriaQuintica

N_JOINTS: int = 5
JOINT_NAMES: List[str] = [f'Joint_{i}' for i in range(1, N_JOINTS + 1)]
MIN_DT: float = 1e-6            # [s] paso mínimo para integrar
EULER_WP_DT_LIMIT: float = 0.5  # cota recomendada de wp*dt para Euler
MODOS_INERCIA = ('constante', 'diagonal', 'completa')
TIPOS_MANDO = ('float64_por_articulacion', 'float64_multiarray', 'joint_state')

# Valores por defecto (los reemplaza config/brazo_adrc.yaml).
DEFAULTS = {
    'frecuencia_control': 200.0,
    'cero_desde_primera_medicion': True,
    'offsets_cero': [3.1415, -1.5707, 0.0, 0.0, 0.0],
    'medicion_absoluta': True,
    'postura_encendido_urdf': [3.1415, -1.5707, 0.0, 0.0, 0.0],
    'wn': [10.0, 10.0, 10.0, 10.0, 1.0],
    'eps': [0.2, 0.2, 0.2, 0.2, 0.2],
    'zeta': [1.5, 1.0, 1.0, 1.5, 1.0],
    'modo_inercia': 'completa',
    'usar_modelo': [True, True, True, True, False],
    'inercia_constante': [1.5, 2.0, 1.3, 0.83, 1.0],
    'armadura': [0.0, 0.0, 0.0, 0.0, 0.0],
    'urdf_brazo': '',
    'feedforward_gravedad': True,
    'feedforward_aceleracion': True,
    'k_tau_hat': [3.75, 3.94, 3.94, 3.75, 1.0],
    'limite_mando': [6.54, 7.35, 7.35, 6.54, 2.0],
    'corriente_vacio': [0.25, 0.497, 0.497, 0.25, 0.0],
    'ancho_comp_friccion': 0.1,
    'margen_corriente_trayectoria': 0.9,
    'v_max_fisica': [1.23, 3.67, 3.67, 1.23, 6.28],
    'a_max_100pct': [2.0, 3.0, 3.0, 2.0, 5.0],
    'factor_100pct': 0.5,
    'velocidad_pct_defecto': 50.0,
    'velocidad_pct_min': 1.0,
    'duracion_minima': 0.5,
    # --- Interfaz ROS (ver sección INTERFAZ del YAML) ---
    'nombres_urdf': JOINT_NAMES,
    'topico_estado': '/world/empty/model/mi_rover/joint_state',
    'nombres_en_estado': JOINT_NAMES,
    'qos_estado_best_effort': False,
    'signo_articulacion': [1.0, 1.0, 1.0, 1.0, 1.0],
    'topico_objetivo_deg': '/brazo/objetivo_deg',
    'topico_objetivo_rad': '/target_joint_states',
    'tipo_mando': 'float64_por_articulacion',
    'topicos_mando': [f'/brazo/{n}/cmd_actuador' for n in JOINT_NAMES],
    'topico_mando_unico': '/brazo/cmd_actuador',
    'nombres_en_mando': JOINT_NAMES,
    'publicar_diagnostico': False,
    'periodo_log': 0.0,
}


class AdrcSimple(Node):
    """Lazo ADRC con trayectoria quíntica e inversa de B por modelo."""

    def __init__(self) -> None:
        """Lee parámetros, calcula ganancias y crea la interfaz ROS."""
        super().__init__('adrc_simple')
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        p = self._read_params()

        # --- Ganancias GPI (en unidades de aceleración) ---
        wn, eps, zeta = p['wn'], p['eps'], p['zeta']
        wp = wn / eps
        g1, g0 = 2.0 * zeta * wn, wn**2
        l1, l0 = 2.0 * zeta * wp, wp**2
        self.k0 = g0 * l0
        self.k1 = l0 * g1 + g0 * l1
        self.k2 = l0 + g0 + l1 * g1
        self.k3 = l1 + g1
        dt = 1.0 / p['frecuencia_control']
        self.names = list(p['nombres_urdf'])
        for name, w, wpdt in zip(self.names, wp, wp * dt):
            if wpdt > EULER_WP_DT_LIMIT:
                self.get_logger().warning(
                    f'{name}: wp={w:.1f} rad/s, wp*dt={wpdt:.2f} > '
                    f'{EULER_WP_DT_LIMIT}; sube frecuencia_control o eps.')

        # --- Actuadores y modelo ---
        self.inv_k_tau = 1.0 / p['k_tau_hat']
        self.cmd_max = p['limite_mando']
        self.i0 = p['corriente_vacio']
        self.fc_gain = 4.0 / max(float(p['ancho_comp_friccion']), 1e-6)
        self.margen_tray = float(p['margen_corriente_trayectoria'])
        self.j_const = p['inercia_constante']
        self.mask = np.asarray(p['usar_modelo'], dtype=bool)
        self.ff_acc = 1.0 if p['feedforward_aceleracion'] else 0.0
        self.ff_grav = bool(p['feedforward_gravedad'])
        self.medicion_absoluta = bool(p['medicion_absoluta'])
        self.postura_urdf = p['postura_encendido_urdf']
        self.modo = p['modo_inercia']
        if self.modo not in MODOS_INERCIA:
            self.get_logger().error(
                f'modo_inercia="{self.modo}" no válido; uso "constante".')
            self.modo = 'constante'
        self.modelo = None
        if self.modo != 'constante' or self.ff_grav:
            self.modelo = self._load_model(p['urdf_brazo'], p['armadura'])
            if self.modelo is None:
                self.modo, self.ff_grav = 'constante', False
        self.m_const = np.diag(self.j_const)

        # --- Trayectoria ---
        self.traj = TrayectoriaQuintica(
            p['factor_100pct'] * p['v_max_fisica'], p['a_max_100pct'],
            p['duracion_minima'])
        self.pct_max = 100.0 / p['factor_100pct']
        self.pct_min = p['velocidad_pct_min']
        self.pct_default = float(np.clip(
            p['velocidad_pct_defecto'], self.pct_min, self.pct_max))
        self.add_on_set_parameters_callback(self._on_set_params)

        # --- Estado ---
        self.zero_offset: Optional[np.ndarray] = (
            None if p['cero_desde_primera_medicion']
            else p['offsets_cero'].copy())
        self.idx: Optional[np.ndarray] = None
        self.n_names: int = -1
        self.q_abs = np.zeros(N_JOINTS)
        self.goal = np.zeros(N_JOINTS)
        self.q_d = np.zeros(N_JOINTS)
        self.qd_d = np.zeros(N_JOINTS)
        self.qdd_d = np.zeros(N_JOINTS)
        self.s1 = np.zeros(N_JOINTS)
        self.s2 = np.zeros(N_JOINTS)
        self.has_measurement = False

        # --- Interfaz ROS (todo configurable desde el YAML) ---
        self._crear_interfaz(p)
        self.diag = bool(p['publicar_diagnostico'])
        self.log_period = float(p['periodo_log'])
        if self.diag:
            self.error_pubs = [
                self.create_publisher(Float64, f'~/error_deg/{name}', 10)
                for name in self.names]
            self.state_pub = self.create_publisher(JointState, '~/state', 10)
            self.reference_pub = self.create_publisher(
                JointState, '~/reference', 10)

        self.last_ns = self.get_clock().now().nanoseconds
        self.create_timer(dt, self._control_loop)
        self.get_logger().info(
            f'ADRC listo a {p["frecuencia_control"]:.0f} Hz | '
            f'modo_inercia={self.modo} | ff_gravedad={self.ff_grav} | '
            f'velocidad por defecto {self.pct_default:.0f} % '
            f'(máx {self.pct_max:.0f} %) | límites {self.cmd_max.round(2)}')

    # ------------------------------------------------------------------
    # Configuración
    # ------------------------------------------------------------------
    def _read_params(self) -> dict:
        """Lee todos los parámetros; los arreglos se validan a 5 valores."""
        p = {}
        for name, default in DEFAULTS.items():
            value = self.get_parameter(name).value
            if isinstance(default, list):
                if len(value) != N_JOINTS:
                    raise ValueError(
                        f'Parámetro {name}: se esperaban {N_JOINTS} valores')
                if isinstance(default[0], float):
                    value = np.asarray(value, dtype=float)
                else:
                    value = list(value)
            p[name] = value
        return p

    def _load_model(self, urdf_path: str, armadura: np.ndarray):
        """Carga el modelo de Pinocchio; devuelve None si no es posible."""
        try:
            from control_adrc_rover.modelo_dinamico import ModeloBrazo
            if not urdf_path:
                from ament_index_python.packages import (
                    get_package_share_directory)
                urdf_path = os.path.join(
                    get_package_share_directory('mi_rover_description'),
                    'urdf', 'brazo.urdf')
            modelo = ModeloBrazo(urdf_path, self.names, armadura)
            self.get_logger().info(f'Modelo dinámico cargado: {urdf_path}')
            return modelo
        except Exception as exc:  # noqa: BLE001 - se informa y se degrada
            self.get_logger().error(
                f'No se pudo cargar el modelo (Pinocchio/URDF): {exc}. '
                f'Se usa modo_inercia="constante" sin gravedad.')
            return None

    def _on_set_params(self, params) -> SetParametersResult:
        """Permite cambiar en vivo la velocidad por defecto."""
        for param in params:
            if param.name == 'velocidad_pct_defecto':
                self.pct_default = float(np.clip(
                    param.value, self.pct_min, self.pct_max))
                self.get_logger().info(
                    f'Velocidad por defecto: {self.pct_default:.0f} %')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Interfaz ROS
    # ------------------------------------------------------------------
    def _crear_interfaz(self, p: dict) -> None:
        """Crea suscriptores y publicadores con los nombres del YAML."""
        self.topico_estado = p['topico_estado']
        self.nombres_estado = list(p['nombres_en_estado'])
        self.signo = np.asarray(p['signo_articulacion'], dtype=float)
        qos_estado = (qos_profile_sensor_data if p['qos_estado_best_effort']
                      else QoSProfile(depth=10))
        self.create_subscription(
            JointState, self.topico_estado, self._state_callback, qos_estado)
        if p['topico_objetivo_deg']:
            self.create_subscription(
                Float64MultiArray, p['topico_objetivo_deg'],
                self._goal_deg_callback, 10)
        if p['topico_objetivo_rad']:
            self.create_subscription(
                JointState, p['topico_objetivo_rad'],
                self._target_rad_callback, 10)

        self.tipo_mando = p['tipo_mando']
        if self.tipo_mando not in TIPOS_MANDO:
            raise ValueError(
                f'tipo_mando="{self.tipo_mando}" no válido; usa uno de '
                f'{TIPOS_MANDO}')
        if self.tipo_mando == 'float64_por_articulacion':
            self.cmd_pubs = [self.create_publisher(Float64, t, 10)
                             for t in p['topicos_mando']]
            self.cmd_msgs = [Float64() for _ in range(N_JOINTS)]
            destino = list(p['topicos_mando'])
        elif self.tipo_mando == 'float64_multiarray':
            self.cmd_pub = self.create_publisher(
                Float64MultiArray, p['topico_mando_unico'], 10)
            self.cmd_msg = Float64MultiArray()
            destino = p['topico_mando_unico']
        else:
            self.cmd_pub = self.create_publisher(
                JointState, p['topico_mando_unico'], 10)
            self.cmd_msg = JointState(name=list(p['nombres_en_mando']))
            destino = p['topico_mando_unico']
        self.get_logger().info(
            f'Interfaz: estado <- {self.topico_estado} '
            f'{self.nombres_estado} | objetivo <- '
            f'{p["topico_objetivo_deg"] or "-"} [°], '
            f'{p["topico_objetivo_rad"] or "-"} [rad] | mando '
            f'({self.tipo_mando}) -> {destino}')

    def _publicar_mando(self, cmd: np.ndarray) -> None:
        """Publica el mando (J1-J4 [A], J5 [N·m]) en el formato elegido."""
        if self.tipo_mando == 'float64_por_articulacion':
            for pub, msg, value in zip(self.cmd_pubs, self.cmd_msgs, cmd):
                msg.data = float(value)
                pub.publish(msg)
        elif self.tipo_mando == 'float64_multiarray':
            self.cmd_msg.data = cmd.tolist()
            self.cmd_pub.publish(self.cmd_msg)
        else:
            self.cmd_msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_msg.effort = cmd.tolist()
            self.cmd_pub.publish(self.cmd_msg)

    # ------------------------------------------------------------------
    # Entradas
    # ------------------------------------------------------------------
    def _state_callback(self, msg: JointState) -> None:
        """Guarda la posición medida y fija el cero en la primera lectura."""
        if self.idx is None or len(msg.name) != self.n_names:
            names = list(msg.name)
            faltan = [n for n in self.nombres_estado if n not in names]
            if faltan:
                self.get_logger().warning(
                    f'{self.topico_estado}: no encuentro {faltan}. Nombres '
                    f'recibidos: {names}. Ajusta "nombres_en_estado" en el '
                    f'YAML.', throttle_duration_sec=5.0)
                return
            self.idx = np.array([names.index(n) for n in self.nombres_estado])
            self.n_names = len(names)
        self.q_abs = self.signo * np.asarray(
            msg.position, dtype=float)[self.idx]
        if self.zero_offset is None:
            self.zero_offset = self.q_abs.copy()
            self.get_logger().info(
                f'Cero fijado en la postura actual: '
                f'{self.zero_offset.round(4)} rad')
        self.has_measurement = True

    def _goal_deg_callback(self, msg: Float64MultiArray) -> None:
        """[J1..J5] en grados (+ % de velocidad opcional); NaN = conservar."""
        data = np.asarray(msg.data, dtype=float)
        if data.size not in (N_JOINTS, N_JOINTS + 1):
            self.get_logger().error(
                f'Se esperaban {N_JOINTS} ángulos [°] y, opcionalmente, '
                f'el % de velocidad; llegaron {data.size} valores.')
            return
        pct = self.pct_default
        if data.size == N_JOINTS + 1 and not np.isnan(data[-1]):
            pct = float(data[-1])
        angles = data[:N_JOINTS]
        goal = np.where(np.isnan(angles), self.goal, np.deg2rad(angles))
        self._new_goal(goal, pct)

    def _target_rad_callback(self, msg: JointState) -> None:
        """Entrada heredada en radianes (admite subconjuntos de joints)."""
        goal = self.goal.copy()
        for name, position in zip(msg.name, msg.position):
            if name in self.names:
                goal[self.names.index(name)] = position
        self._new_goal(goal, self.pct_default)

    def _new_goal(self, goal: np.ndarray, pct: float) -> None:
        """Planea la trayectoria desde el estado actual de la referencia."""
        if not self.has_measurement:
            self.get_logger().warning(
                'Aún no hay medición del brazo; objetivo ignorado.')
            return
        pct_ok = float(np.clip(pct, self.pct_min, self.pct_max))
        if pct_ok != pct:
            self.get_logger().warning(
                f'Velocidad {pct:.0f} % fuera de rango; se usa {pct_ok:.0f} %')
        self.goal = goal
        t_now = self.get_clock().now().nanoseconds * 1e-9
        q0, v0, a0 = self.q_d.copy(), self.qd_d.copy(), self.qdd_d.copy()
        escala = pct_ok / 100.0
        T = self.traj.planear(t_now, q0, v0, a0, goal, escala)
        alargada = False
        for _ in range(40):
            if self._trayectoria_factible():
                break
            escala /= 1.15
            T = self.traj.planear(t_now, q0, v0, a0, goal, escala)
            alargada = True
        aviso = (' — alargada para no exceder la corriente'
                 if alargada else '')
        self.get_logger().info(
            f'Nuevo objetivo: {np.rad2deg(goal).round(2)}° a {pct_ok:.0f} % '
            f'(duración {T:.2f} s){aviso}')

    def _q_modelo(self, q_rel: np.ndarray) -> np.ndarray:
        """Convierte un ángulo relativo a la convención del URDF."""
        q_abs = q_rel + self.zero_offset
        return q_abs if self.medicion_absoluta else q_abs + self.postura_urdf

    def _trayectoria_factible(self, n_muestras: int = 40) -> bool:
        """True si la corriente nominal de la curva cabe en el margen."""
        if self.modelo is None:
            return True
        limite = self.margen_tray * self.cmd_max
        for t in np.linspace(0.0, self.traj.duration, n_muestras):
            q, _, qdd = self.traj.evaluar(t)
            m_q, g_q = self.modelo.calcular(self._q_modelo(q))
            i_nom = np.abs((m_q @ qdd + g_q) * self.inv_k_tau) + self.i0
            if np.any((i_nom > limite) & self.mask):
                return False
        return True

    # ------------------------------------------------------------------
    # Lazo de control
    # ------------------------------------------------------------------
    def _inertia_and_gravity(self):
        """M_hat(q) según el modo, con las articulaciones sin modelo aparte."""
        if self.modelo is None:
            return self.m_const, None
        m_q, g_q = self.modelo.calcular(
            self._q_modelo(self.q_abs - self.zero_offset))
        if self.modo == 'constante':
            m_hat = self.m_const
        else:
            m_hat = m_q if self.modo == 'completa' else np.diag(np.diag(m_q))
            off = ~self.mask
            if off.any():
                m_hat = m_hat.copy()
                m_hat[off, :] = 0.0
                m_hat[:, off] = 0.0
                m_hat[off, off] = self.j_const[off]
        g_hat = np.where(self.mask, g_q, 0.0) if self.ff_grav else None
        return m_hat, g_hat

    def _control_loop(self) -> None:
        """Muestrea la trayectoria y calcula el mando."""
        now = self.get_clock().now().nanoseconds
        dt = (now - self.last_ns) * 1e-9
        self.last_ns = now
        if not self.has_measurement or dt <= MIN_DT:
            return

        if self.traj.active:
            self.q_d, self.qd_d, self.qdd_d = self.traj.muestrear(now * 1e-9)
            if not self.traj.active:
                self.get_logger().info('Trayectoria completada.')

        q_rel = self.q_abs - self.zero_offset
        error = q_rel - self.q_d
        s2_dot = error - self.k3 * self.s2
        v = (self.ff_acc * self.qdd_d
             - (self.k2 * s2_dot + self.k1 * self.s2 + self.k0 * self.s1))

        m_hat, g_hat = self._inertia_and_gravity()
        tau = m_hat @ v
        if g_hat is not None:
            tau = tau + g_hat
        cmd_free = tau * self.inv_k_tau
        cmd_core = np.clip(cmd_free, -self.cmd_max, self.cmd_max)

        # Anti-windup: la integral se congela mientras haya saturación.
        self.s1 += np.where(cmd_core == cmd_free, self.s2 * dt, 0.0)
        self.s2 += s2_dot * dt

        # Compensación de la corriente en vacío (fricción del motor).
        cmd = np.clip(cmd_core + self.i0 * np.tanh(self.fc_gain * cmd_core),
                      -self.cmd_max, self.cmd_max)

        self._publicar_mando(self.signo * cmd)

        if self.diag:
            self._publish_diagnostics(q_rel, error, cmd)
        if self.log_period > 0.0:
            self.get_logger().info(
                f'q*[°]={np.rad2deg(self.q_d).round(1)} '
                f'q[°]={np.rad2deg(q_rel).round(1)} '
                f'e[°]={np.rad2deg(error).round(2)} '
                f'i[A] (J5 N·m)={cmd.round(2)}',
                throttle_duration_sec=self.log_period)

    def _publish_diagnostics(
            self, q_rel: np.ndarray, error: np.ndarray,
            cmd: np.ndarray) -> None:
        """Publica error [°], estado (effort = mando) y referencia."""
        for pub, err_deg in zip(self.error_pubs, np.rad2deg(error)):
            pub.publish(Float64(data=float(err_deg)))
        stamp = self.get_clock().now().to_msg()
        state = JointState(name=self.names, position=q_rel.tolist(),
                           effort=cmd.tolist())
        state.header.stamp = stamp
        self.state_pub.publish(state)
        reference = JointState(
            name=self.names, position=self.q_d.tolist(),
            velocity=self.qd_d.tolist(), effort=self.qdd_d.tolist())
        reference.header.stamp = stamp
        self.reference_pub.publish(reference)


def main(args: Optional[List[str]] = None) -> None:
    """Punto de entrada del nodo."""
    rclpy.init(args=args)
    node = AdrcSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
