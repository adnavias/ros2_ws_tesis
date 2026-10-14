"""Nodo ROS 2 del control del brazo (lifecycle node "adrc_simple").

Capa delgada sobre nucleo.controlador.Controlador: lee la configuración
(perfil de brazo + plataforma + ajustes guardados), conecta tópicos,
servicios y acciones, y ejecuta el lazo en cada medición nueva.

Ciclo de vida (configure / activate / deactivate / cleanup). Con
`autoarranque: true` (por defecto) el nodo se configura y activa solo,
así que funciona igual que antes sin ninguna interfaz:
    ros2 launch control_adrc_rover brazo.launch.py sim:=true

Hilos. Se usa un MultiThreadedExecutor con grupos de callbacks separados:
    - medición + lazo (exclusivo): cada JointState ejecuta un paso,
    - watchdog (exclusivo, 100 Hz): medición perdida y heartbeat,
    - servicios, acciones, objetivos y parámetros (reentrante): el
      replaneo y el autotuning nunca bloquean el lazo.

Interfaces (prefijo ~ = /adrc_simple/):
    Tópicos de entrada
        <interfaz.topico_estado>       sensor_msgs/JointState  medición [rad]
        /brazo/objetivo_deg            std_msgs/Float64MultiArray [°] (+ %)
        /brazo/objetivo_nombres_deg    sensor_msgs/JointState  nombres + [°]
                                       (velocity[0] opcional = %)
        /target_joint_states           sensor_msgs/JointState  [rad] (heredado)
        /brazo/paro_emergencia         std_msgs/Bool (true = paro)
        /brazo/interfaz/heartbeat      std_msgs/Empty (modo calibración)
    Tópicos de salida
        mando (según interfaz.tipo_mando), ~/estado (EstadoControl),
        /brazo/paro_emergencia/estado  std_msgs/Bool latched
        ~/error_deg/<J>, ~/state, ~/reference (diagnóstico heredado)
    Servicios
        ~/habilitar (SetBool), ~/paro_emergencia (Trigger),
        ~/liberar_paro (Trigger), ~/modo_calibracion (SetBool),
        ~/cargar_perfil (CargarPerfil), ~/guardar_configuracion
        (GuardarConfiguracion), ~/estimar_modo (Trigger)
    Acciones
        ~/mover_a (MoverA), ~/autotune (Autotune)
    Parámetros dinámicos (validados)
        <J>.polos, <J>.beta, <J>.alfa, <J>.armadura, <J>.alfa_gravedad,
        <J>.friccion_coulomb, <J>.limite_mando, velocidad_pct_defecto,
        shaper.tipo, shaper.frecuencia_hz, shaper.amortiguamiento,
        shaper.tolerancia_ei, supervisor.habilitado

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import math
import os
import threading
import time
from collections import deque
from typing import Dict, List, Optional

import numpy as np
import rclpy
import yaml
from rcl_interfaces.msg import (FloatingPointRange, ParameterDescriptor,
                                SetParametersResult)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from control_adrc_interfaces.action import Autotune as AutotuneAccion
from control_adrc_interfaces.action import MoverA
from control_adrc_interfaces.msg import EstadoControl
from control_adrc_interfaces.srv import CargarPerfil, GuardarConfiguracion
from control_adrc_rover.nucleo import config as cfgmod
from control_adrc_rover.nucleo.autotune import Autotune
from control_adrc_rover.nucleo.controlador import Controlador
from control_adrc_rover.nucleo.shaper import estimar_modo
from control_adrc_rover.rutas import ruta_ajustes, ruta_capa

try:  # rclpy de Humble trae lifecycle; si no, se emula con un Node normal
    from rclpy.lifecycle import Node as NodoBase
    from rclpy.lifecycle import State, TransitionCallbackReturn
    HAY_LIFECYCLE = True
except ImportError:  # pragma: no cover
    from rclpy.node import Node as NodoBase
    State = object
    HAY_LIFECYCLE = False

    class TransitionCallbackReturn:  # noqa: D101
        SUCCESS = 'success'
        FAILURE = 'failure'
        ERROR = 'error'

# Rangos válidos de los parámetros editables por articulación
RANGOS = {'beta': (0.05, 1.5), 'alfa': (0.05, 20.0), 'armadura': (0.0, 20.0),
          'alfa_gravedad': (0.0, 3.0), 'friccion_coulomb': (0.0, 5.0),
          'limite_mando': (0.01, 1e9)}
CAMPOS_ARTICULACION = tuple(RANGOS)


class NodoControl(NodoBase):
    """Lifecycle node del control del brazo."""

    def __init__(self) -> None:
        """Declara parámetros base y crea las interfaces permanentes."""
        super().__init__('adrc_simple')
        self.declare_parameter('brazo', 'principal')
        self.declare_parameter('plataforma', 'sim')
        self.declare_parameter('archivo_ajustes', '')
        self.declare_parameter('montadas', 0)
        self.declare_parameter('estados', '')
        self.declare_parameter('autoarranque', True)

        self.lock = threading.RLock()
        self.lock_plan = threading.Lock()
        self.cb_lazo = MutuallyExclusiveCallbackGroup()
        self.cb_watchdog = MutuallyExclusiveCallbackGroup()
        self.cb_srv = ReentrantCallbackGroup()

        self.cfg: Optional[cfgmod.ConfigBrazo] = None
        self.ctrl: Optional[Controlador] = None
        self.activo = False
        self.modo_calibracion = False
        self.t_heartbeat = time.monotonic()
        self.t_rx: Optional[float] = None
        self.autotune: Optional[Autotune] = None
        self._recursos: List = []
        self._param_dinamicos: List[str] = []
        self._idx_estado: Optional[np.ndarray] = None
        self._nombres_msg: Optional[tuple] = None
        self._n_diag = 1
        self._k_paso = 0
        self._imu: deque = deque(maxlen=4000)
        self._ultimo_log = 0.0

        qos_latch = QoSProfile(depth=1,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.pub_paro = self.create_publisher(
            Bool, '/brazo/paro_emergencia/estado', qos_latch)
        self.pub_paro.publish(Bool(data=False))
        self.create_subscription(Bool, '/brazo/paro_emergencia',
                                 self._cb_paro_topico, 10,
                                 callback_group=self.cb_srv)
        srv = [
            (SetBool, '~/habilitar', self._srv_habilitar),
            (Trigger, '~/paro_emergencia', self._srv_paro),
            (Trigger, '~/liberar_paro', self._srv_liberar),
            (SetBool, '~/modo_calibracion', self._srv_calibracion),
            (CargarPerfil, '~/cargar_perfil', self._srv_cargar_perfil),
            (GuardarConfiguracion, '~/guardar_configuracion',
             self._srv_guardar),
            (Trigger, '~/estimar_modo', self._srv_estimar_modo),
        ]
        for tipo, nombre, cb in srv:
            self.create_service(tipo, nombre, cb, callback_group=self.cb_srv)
        self._accion_mover = ActionServer(
            self, MoverA, '~/mover_a', execute_callback=self._exec_mover,
            goal_callback=self._goal_ok, cancel_callback=self._cancel_ok,
            callback_group=self.cb_srv)
        self._accion_autotune = ActionServer(
            self, AutotuneAccion, '~/autotune',
            execute_callback=self._exec_autotune,
            goal_callback=self._goal_ok, cancel_callback=self._cancel_ok,
            callback_group=self.cb_srv)
        self.add_on_set_parameters_callback(self._on_set_params)
        self.create_timer(0.01, self._cb_watchdog,
                          callback_group=self.cb_watchdog)
        if not HAY_LIFECYCLE:
            self.get_logger().warning(
                'rclpy sin soporte lifecycle: se emula configure/activate')

    # ==================================================================
    # Ciclo de vida
    # ==================================================================
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Carga la configuración, el modelo y crea la interfaz."""
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        try:
            brazo = str(gp('brazo'))
            perfil = ruta_capa(brazo, 'brazos')
            plataforma = ruta_capa(str(gp('plataforma')), 'plataformas')
            ajustes = str(gp('archivo_ajustes')) or ''
            nombre_brazo = cfgmod.leer_yaml(perfil).get('nombre', brazo)
            if not ajustes:
                ajustes = ruta_ajustes(nombre_brazo)
            self.archivo_ajustes = ajustes
            estados = cfgmod.parsear_estados(str(gp('estados')))
            cfg = cfgmod.cargar(perfil, plataforma, ajustes,
                                int(gp('montadas')), estados)
            modelo = None
            try:
                from control_adrc_rover.nucleo.modelo import ModeloBrazo
                modelo = ModeloBrazo(cfg.urdf, cfg)
                self.get_logger().info(
                    f'Modelo dinámico: {cfg.urdf} | coordenadas libres '
                    f'{modelo.nombres}')
            except cfgmod.ErrorConfig:
                raise
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f'No se pudo cargar el modelo (Pinocchio/URDF): {exc}. '
                    f'Se sigue con inercia constante y sin gravedad.')
            ctrl = Controlador(cfg, modelo, log=self.get_logger().info,
                               aviso=self.get_logger().warning)
        except cfgmod.ErrorConfig as exc:
            self.get_logger().error(f'Configuración inválida: {exc}')
            return TransitionCallbackReturn.FAILURE
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Error al configurar: {exc}')
            return TransitionCallbackReturn.FAILURE
        with self.lock:
            self.cfg, self.ctrl = cfg, ctrl
            self._idx_estado = None
            self.t_rx = None
        self.plataforma = str(gp('plataforma'))
        self._crear_interfaz(cfg)
        self._declarar_dinamicos(cfg)
        self.get_logger().info(
            f'Configurado: brazo "{cfg.nombre}" | {cfg.resumen()} | '
            f'archivos {cfg.archivos}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Empieza a procesar mediciones (el cero se fija con la primera)."""
        self.activo = True
        self.get_logger().info(
            f'Activo a {self.cfg.lazo["frecuencia_control"]:.0f} Hz; esperando '
            f'medición en {self.cfg.interfaz["topico_estado"]}')
        return (super().on_activate(state) if HAY_LIFECYCLE
                else TransitionCallbackReturn.SUCCESS)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deja de controlar (mando cero) sin destruir la configuración."""
        self.activo = False
        with self.lock:
            if self.ctrl is not None:
                self.ctrl.deshabilitar('nodo desactivado')
                if self.autotune and not self.autotune.terminado:
                    self.autotune.cancelar('nodo desactivado')
                self._publicar_cero()
        return (super().on_deactivate(state) if HAY_LIFECYCLE
                else TransitionCallbackReturn.SUCCESS)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destruye la interfaz dependiente del perfil."""
        self._destruir_interfaz()
        with self.lock:
            self.cfg, self.ctrl = None, None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Mando cero al apagar."""
        self.activo = False
        with self.lock:
            if self.ctrl is not None:
                self._publicar_cero()
        return TransitionCallbackReturn.SUCCESS

    def _transicion(self, nombre: str) -> None:
        """Ejecuta la transición lifecycle `nombre` (configure, activate,
        deactivate, cleanup); sin lifecycle llama al callback directo."""
        trigger = getattr(self, f'trigger_{nombre}', None) if HAY_LIFECYCLE \
            else None
        if trigger is not None:
            trigger()
        else:
            getattr(self, f'on_{nombre}')(None)

    def arrancar(self) -> None:
        """configure + activate (autoarranque)."""
        self._transicion('configure')
        if self.ctrl is not None:
            self._transicion('activate')

    # ==================================================================
    # Interfaz dependiente del perfil
    # ==================================================================
    def _crear_interfaz(self, cfg: cfgmod.ConfigBrazo) -> None:
        itf = cfg.interfaz
        ctrl = self.ctrl
        r = self._recursos
        qos = (qos_profile_sensor_data if itf['qos_estado_best_effort']
               else QoSProfile(depth=10))
        r.append(self.create_subscription(
            JointState, itf['topico_estado'], self._cb_estado, qos,
            callback_group=self.cb_lazo))
        if itf.get('topico_objetivo_deg'):
            r.append(self.create_subscription(
                Float64MultiArray, itf['topico_objetivo_deg'],
                self._cb_objetivo_deg, 10, callback_group=self.cb_srv))
        if itf.get('topico_objetivo_rad'):
            r.append(self.create_subscription(
                JointState, itf['topico_objetivo_rad'],
                self._cb_objetivo_rad, 10, callback_group=self.cb_srv))
        if itf.get('topico_objetivo_nombres_deg'):
            r.append(self.create_subscription(
                JointState, itf['topico_objetivo_nombres_deg'],
                self._cb_objetivo_nombres, 10, callback_group=self.cb_srv))
        if itf.get('topico_heartbeat'):
            r.append(self.create_subscription(
                Empty, itf['topico_heartbeat'], self._cb_heartbeat, 10,
                callback_group=self.cb_srv))
        if itf.get('topico_imu'):
            r.append(self.create_subscription(
                Imu, itf['topico_imu'], self._cb_imu,
                qos_profile_sensor_data, callback_group=self.cb_srv))

        # Mando
        self.tipo_mando = itf['tipo_mando']
        art_crudo = cfg.crudo.get('articulaciones', {})
        destinos = []
        if self.tipo_mando == 'float64_por_articulacion':
            self.pubs_mando = []
            for a in ctrl.movibles:
                topico = (art_crudo.get(a.nombre, {}) or {}).get(
                    'topico_mando') or itf['patron_topico_mando'].format(
                        articulacion=a.nombre_en_mando)
                pub = self.create_publisher(Float64, topico, 10)
                self.pubs_mando.append(pub)
                r.append(pub)
                destinos.append(topico)
        else:
            tipo = (Float64MultiArray if self.tipo_mando ==
                    'float64_multiarray' else JointState)
            self.pub_mando = self.create_publisher(
                tipo, itf['topico_mando_unico'], 10)
            r.append(self.pub_mando)
            destinos.append(itf['topico_mando_unico'])

        # Diagnóstico
        diag = cfg.diagnostico
        self.pub_estado = self.create_publisher(EstadoControl, '~/estado', 10)
        r.append(self.pub_estado)
        self.diag_heredado = bool(diag['publicar'])
        if self.diag_heredado:
            self.pubs_error = [self.create_publisher(
                Float64, f'~/error_deg/{n}', 10) for n in ctrl.nombres]
            self.pub_state = self.create_publisher(JointState, '~/state', 10)
            self.pub_ref = self.create_publisher(JointState, '~/reference',
                                                 10)
            r.extend(self.pubs_error + [self.pub_state, self.pub_ref])
        f_ctrl = float(cfg.lazo['frecuencia_control'])
        self._n_diag = max(1, int(round(f_ctrl / float(diag['frecuencia']))))
        self.periodo_log = float(diag['periodo_log'])
        self.get_logger().info(
            f'Interfaz: estado <- {itf["topico_estado"]} | mando '
            f'({self.tipo_mando}) -> {destinos} | objetivos <- '
            f'{itf.get("topico_objetivo_deg")} [°], '
            f'{itf.get("topico_objetivo_nombres_deg")} [nombres, °], '
            f'{itf.get("topico_objetivo_rad")} [rad]')

    def _destruir_interfaz(self) -> None:
        for rec in self._recursos:
            try:
                if hasattr(rec, 'topic_name') and hasattr(rec, 'publish'):
                    self.destroy_publisher(rec)
                else:
                    self.destroy_subscription(rec)
            except Exception:  # noqa: BLE001
                pass
        self._recursos = []
        for nombre in self._param_dinamicos:
            try:
                self.undeclare_parameter(nombre)
            except Exception:  # noqa: BLE001
                pass
        self._param_dinamicos = []

    # ==================================================================
    # Parámetros dinámicos
    # ==================================================================
    def _declarar(self, nombre: str, valor, descripcion: str,
                  rango=None) -> None:
        desc = ParameterDescriptor(description=descripcion)
        if rango is not None:
            desc.floating_point_range = [FloatingPointRange(
                from_value=float(rango[0]), to_value=float(rango[1]),
                step=0.0)]
        if self.has_parameter(nombre):
            self.undeclare_parameter(nombre)
        self.declare_parameter(nombre, valor, desc)
        self._param_dinamicos.append(nombre)

    def _declarar_dinamicos(self, cfg: cfgmod.ConfigBrazo) -> None:
        c = self.ctrl
        self._cargando_params = True
        try:
            self._declarar('velocidad_pct_defecto', float(c.pct_defecto),
                           '% de velocidad por defecto',
                           (c.pct_min, c.pct_max))
            for g, k in enumerate(c.k_gpi):
                a = c.movibles[k]
                n = a.nombre
                self._declarar(f'{n}.polos', [float(p) for p in
                                              c.gpi.polos[g]],
                               'Polos reales λ1..λ4 del GPI [rad/s]')
                self._declarar(f'{n}.beta', float(c.beta[g]),
                               'Margen de inercia (control con beta*M)',
                               RANGOS['beta'])
                self._declarar(f'{n}.alfa', float(c.alfa[g]),
                               'Escala de la inercia de los eslabones',
                               (0.05, 20.0))
                self._declarar(f'{n}.armadura', float(c.armadura[g]),
                               'Armadura (inercia del rotor) [kg m^2]',
                               (0.0, 20.0))
                self._declarar(f'{n}.alfa_gravedad', float(c.alfa_g[g]),
                               'Escala de la gravedad', (0.0, 3.0))
                self._declarar(f'{n}.friccion_coulomb', float(c.fc[g]),
                               'Compensación de fricción [A o N·m]',
                               (0.0, 5.0))
                self._declarar(f'{n}.limite_mando', float(c.limite[g]),
                               'Límite del mando [A o N·m]',
                               (0.01, self._limite_max(a)))
            sh = cfg.shaper
            self._declarar('shaper.tipo', str(sh['tipo']),
                           'ninguno | zv | zvd | ei')
            self._declarar('shaper.frecuencia_hz', float(sh['frecuencia_hz']),
                           'Frecuencia del modo [Hz]', (0.05, 50.0))
            self._declarar('shaper.amortiguamiento',
                           float(sh['amortiguamiento']),
                           'Amortiguamiento del modo', (0.0, 0.9))
            self._declarar('shaper.tolerancia_ei', float(sh['tolerancia_ei']),
                           'Vibración residual tolerada (EI)', (0.0, 0.3))
            self._declarar('supervisor.habilitado',
                           bool(cfg.supervisor['habilitado']),
                           'Supervisor que baja wn si hay sobreimpulso')
        finally:
            self._cargando_params = False

    def _limite_max(self, a: cfgmod.Articulacion) -> float:
        """Límite máximo permitido (corriente continua y par del reductor)."""
        motor = (self.cfg.crudo.get('motores', {}) or {}).get(a.motor, {})
        tope = float(motor.get('corriente_continua', a.limite_mando * 1.0))
        par = motor.get('par_continuo_salida')
        if par is not None and a.k_tau > 0:
            tope = min(tope, float(par) / a.k_tau)
        return max(tope, a.limite_mando)

    def _on_set_params(self, params) -> SetParametersResult:
        if getattr(self, '_cargando_params', False):
            return SetParametersResult(successful=True)
        c = self.ctrl
        cambios = []
        for p in params:
            nombre = p.name
            if nombre in ('brazo', 'plataforma', 'archivo_ajustes',
                          'montadas', 'estados', 'autoarranque',
                          'use_sim_time'):
                continue          # se aplican en el próximo configure
            if c is None:
                return SetParametersResult(
                    successful=False, reason='Nodo sin configurar')
            try:
                if nombre == 'velocidad_pct_defecto':
                    v = float(p.value)
                    if not c.pct_min <= v <= c.pct_max:
                        raise ValueError(f'fuera de [{c.pct_min}, '
                                         f'{c.pct_max}]')
                    cambios.append(lambda v=v: setattr(c, 'pct_defecto', v))
                elif nombre.startswith('shaper.'):
                    cambios.append(self._cambio_shaper(nombre, p.value))
                elif nombre == 'supervisor.habilitado':
                    cambios.append(lambda v=bool(p.value): setattr(
                        c.supervisor, 'habilitado', v))
                elif '.' in nombre:
                    art, campo = nombre.split('.', 1)
                    cambios.append(self._cambio_articulacion(art, campo,
                                                             p.value))
            except (ValueError, KeyError, RuntimeError) as exc:
                return SetParametersResult(successful=False,
                                           reason=f'{nombre}: {exc}')
        with self.lock:
            for aplicar in cambios:
                if aplicar is not None:
                    aplicar()
        return SetParametersResult(successful=True)

    def _cambio_articulacion(self, art: str, campo: str, valor):
        c = self.ctrl
        if art not in c.nombres:
            return None
        a = c.movibles[c.nombres.index(art)]
        if campo == 'polos':
            polos = [float(x) for x in valor]
            if len(polos) != 4 or any(x <= 0.0 for x in polos):
                raise ValueError('se necesitan 4 polos positivos')
            if max(polos) * c.dt_nominal > 0.5:
                raise ValueError(f'polo {max(polos):.1f} rad/s demasiado '
                                 f'rápido para {1 / c.dt_nominal:.0f} Hz '
                                 f'(λ·dt > 0.5)')
            if min(polos) < 0.5:
                raise ValueError('el polo más lento debe ser >= 0.5 rad/s')
            return lambda: c.fijar_polos(art, polos)
        if campo in CAMPOS_ARTICULACION:
            v = float(valor)
            lo, hi = RANGOS[campo]
            if not (lo <= v <= hi) or math.isnan(v):
                raise ValueError(f'fuera de [{lo}, {hi}]')
            if campo == 'limite_mando' and v > self._limite_max(a) + 1e-9:
                raise ValueError(f'supera el máximo del motor '
                                 f'({self._limite_max(a):.2f})')
            if campo == 'beta' and v < 0.6:
                self.get_logger().warning(
                    f'{art}: beta={v} < 0.6; con kappa bajo el lazo pierde '
                    f'amortiguamiento (ver guía)')
            return lambda: c.fijar(art, campo, v)
        raise ValueError(f'campo desconocido "{campo}"')

    def _cambio_shaper(self, nombre: str, valor):
        c = self.ctrl
        sh = dict(c.cfg.shaper)
        clave = nombre.split('.', 1)[1]
        if clave == 'tipo':
            if valor not in cfgmod.SHAPERS:
                raise ValueError(f'usa uno de {cfgmod.SHAPERS}')
            sh['tipo'] = valor
        else:
            sh[clave] = float(valor)
        if c.traj.active:
            raise RuntimeError('espera a que el brazo esté quieto')
        return lambda: c.configurar_shaper(sh['tipo'], sh['frecuencia_hz'],
                                           sh['amortiguamiento'],
                                           sh['tolerancia_ei'])

    # ==================================================================
    # Medición y lazo
    # ==================================================================
    def _cb_estado(self, msg: JointState) -> None:
        if not self.activo:
            return
        ctrl = self.ctrl
        if ctrl is None:
            return
        nombres = tuple(msg.name)
        if self._idx_estado is None or nombres != self._nombres_msg:
            buscados = [ctrl.movibles[k].nombre_en_estado
                        for k in ctrl.k_med]
            faltan = [n for n in buscados if n not in nombres]
            if faltan:
                self.get_logger().warning(
                    f'{self.cfg.interfaz["topico_estado"]}: no encuentro '
                    f'{faltan}. Nombres recibidos: {list(nombres)}. Ajusta '
                    f'"nombre_en_estado" en la plataforma.',
                    throttle_duration_sec=5.0)
                return
            self._idx_estado = np.array([nombres.index(n) for n in buscados],
                                        dtype=int)
            self._nombres_msg = nombres
        if len(msg.position) < len(nombres):
            return
        raw = np.asarray(msg.position, dtype=float)[self._idx_estado]
        t = self._tiempo_medicion(msg)
        ahora = time.monotonic()
        with self.lock:
            if ctrl.t_med is not None and t < ctrl.t_med - 1.0:
                self.get_logger().warning('El tiempo de la medición retrocedió '
                                          '(¿reinicio de Gazebo?); se '
                                          'reinicia el reloj del lazo')
                ctrl.t_med = None
                ctrl.t_prev = None
            self.t_rx = ahora
            if not ctrl.medir(raw, t):
                return
            mando = ctrl.paso(t)
            self._publicar_mando(mando)
            self._k_paso += 1
            if self._k_paso % self._n_diag == 0:
                self._publicar_diagnostico()
        self._log_periodico()

    def _tiempo_medicion(self, msg: JointState) -> float:
        st = msg.header.stamp
        t = st.sec + st.nanosec * 1e-9
        if self.cfg.interfaz.get('usar_stamp_medicion', True) and t > 0.0:
            return t
        return self.get_clock().now().nanoseconds * 1e-9

    def _cb_watchdog(self) -> None:
        if not self.activo or self.ctrl is None:
            return
        ahora = time.monotonic()
        with self.lock:
            ctrl = self.ctrl
            if self.modo_calibracion and ctrl.estado in ('habilitado',
                                                         'deshabilitado'):
                edad_hb = ahora - self.t_heartbeat
                if edad_hb > float(self.cfg.seguridad['heartbeat_timeout']):
                    ctrl.paro(f'se perdió el heartbeat de la interfaz '
                              f'({edad_hb:.2f} s) en modo calibración')
                    if self.autotune and not self.autotune.terminado:
                        self.autotune.cancelar('paro por heartbeat')
                    self._publicar_paro()
            if self.t_rx is None:
                return
            mando = ctrl.paso_sin_medicion(ahora - self.t_rx)
            if mando is not None:
                self._publicar_mando(mando)
                if self._k_paso % self._n_diag == 0:
                    self._publicar_diagnostico()
                self._k_paso += 1

    def _publicar_mando(self, mando) -> None:
        ctrl = self.ctrl
        if self.tipo_mando == 'float64_por_articulacion':
            for pub, val, pubb in zip(self.pubs_mando, mando.valores,
                                      mando.publicar):
                if pubb:
                    pub.publish(Float64(data=float(val)))
        elif self.tipo_mando == 'float64_multiarray':
            vals = np.where(mando.publicar, mando.valores, np.nan)
            self.pub_mando.publish(Float64MultiArray(data=vals.tolist()))
        else:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            for a, val, pubb in zip(ctrl.movibles, mando.valores,
                                    mando.publicar):
                if not pubb:
                    continue
                msg.name.append(a.nombre_en_mando)
                if a.mando == 'posicion_directa':
                    msg.position.append(float(val))
                    msg.effort.append(float('nan'))
                else:
                    msg.position.append(float('nan'))
                    msg.effort.append(float(val))
            self.pub_mando.publish(msg)

    def _publicar_cero(self) -> None:
        ctrl = self.ctrl
        from control_adrc_rover.nucleo.controlador import Mando
        valores = np.zeros(ctrl.n)
        publicar = np.ones(ctrl.n, dtype=bool)
        publicar[ctrl.k_pos] = False
        try:
            self._publicar_mando(Mando(valores, publicar,
                                       [a.mando for a in ctrl.movibles]))
        except Exception:  # noqa: BLE001 - publicadores ya destruidos
            pass

    def _publicar_diagnostico(self) -> None:
        ctrl = self.ctrl
        d = ctrl.diagnostico()
        m = EstadoControl()
        m.header.stamp = self.get_clock().now().to_msg()
        m.brazo = self.cfg.nombre
        m.plataforma = self.plataforma
        m.estado = d.estado
        m.motivo = ctrl.motivo
        m.watchdog = d.watchdog
        m.paro_emergencia = ctrl.estado == 'paro'
        m.modo_calibracion = self.modo_calibracion
        m.trayectoria_activa = d.trayectoria_activa
        at = self.autotune
        m.autotune_activo = bool(at and not at.terminado)
        m.autotune_progreso = float(at.progreso) if at else 0.0
        m.autotune_etapa = at.etapa if at else ''
        m.rampa_gravedad = float(d.rampa_gravedad)
        m.nombres_perfil = self.cfg.nombres
        m.estados_perfil = [a.estado for a in self.cfg.articulaciones]
        m.nombres = list(ctrl.nombres)
        m.mando = [a.mando for a in ctrl.movibles]
        f = lambda x: [float(v) for v in x]  # noqa: E731
        m.posicion_deg = f(np.degrees(d.q))
        m.referencia_deg = f(np.degrees(d.q_ref))
        m.error_deg = f(np.degrees(d.error))
        m.mando_valor = f(d.mando)
        lim = np.full(ctrl.n, np.nan)
        lim[ctrl.k_gpi] = ctrl.limite * ctrl.factor_limite
        m.limite_mando = f(lim)
        m.phi_hat = f(d.phi_hat)
        m.kappa = f(d.kappa)
        m.wn = f(d.wn)
        polos = np.full((ctrl.n, 4), np.nan)
        polos[ctrl.k_gpi] = ctrl.gpi.polos
        m.polos = f(polos.ravel())
        m.beta = f(d.beta)
        for campo, arr in (('alfa', ctrl.alfa), ('armadura', ctrl.armadura),
                           ('alfa_gravedad', ctrl.alfa_g),
                           ('friccion_coulomb', ctrl.fc)):
            v = np.full(ctrl.n, np.nan)
            v[ctrl.k_gpi] = arr
            setattr(m, campo, f(v))
        m.sobreimpulso_deg = f(np.degrees(d.sobreimpulso))
        m.saturaciones = [int(x) for x in d.saturaciones]
        m.saturado = [bool(x) for x in d.saturado]
        m.velocidad_pct_defecto = float(ctrl.pct_defecto)
        m.supervisor_habilitado = bool(ctrl.supervisor.habilitado)
        m.shaper_tipo = ctrl.shaper.tipo
        m.shaper_frecuencia_hz = float(ctrl.shaper.frecuencia_hz)
        m.shaper_amortiguamiento = float(ctrl.shaper.zeta)
        m.eventos = ctrl.eventos[-12:]
        self.pub_estado.publish(m)
        if self.diag_heredado:
            for pub, e in zip(self.pubs_error, np.degrees(d.error)):
                pub.publish(Float64(data=float(e)))
            st = JointState(name=list(ctrl.nombres), position=f(d.q),
                            effort=f(d.mando))
            st.header.stamp = m.header.stamp
            self.pub_state.publish(st)
            ref = JointState(name=list(ctrl.nombres), position=f(d.q_ref),
                             velocity=f(ctrl.qd_d), effort=f(ctrl.qdd_d))
            ref.header.stamp = m.header.stamp
            self.pub_ref.publish(ref)

    def _publicar_paro(self) -> None:
        activo = self.ctrl is not None and self.ctrl.estado == 'paro'
        self.pub_paro.publish(Bool(data=activo))

    def _log_periodico(self) -> None:
        if self.periodo_log <= 0.0:
            return
        ahora = time.monotonic()
        if ahora - self._ultimo_log < self.periodo_log:
            return
        self._ultimo_log = ahora
        c = self.ctrl
        d = c.diagnostico()
        self.get_logger().info(
            f'[{d.estado}] q*[°]={np.degrees(d.q_ref).round(1).tolist()} '
            f'q[°]={np.degrees(d.q).round(1).tolist()} '
            f'e[°]={np.degrees(d.error).round(2).tolist()} '
            f'mando={np.round(d.mando, 2).tolist()}')

    # ==================================================================
    # Objetivos
    # ==================================================================
    def _mover(self, objetivo_usr: np.ndarray, pct: Optional[float]):
        """Planea fuera del lazo y aplica con lock. Devuelve (ok, msg)."""
        ctrl = self.ctrl
        if ctrl is None or not self.activo:
            return False, 'El control no está activo'
        if self.autotune and not self.autotune.terminado:
            return False, 'Autotuning en curso: objetivo ignorado'
        with self.lock_plan:
            ok, msg, tr = ctrl.planear(objetivo_usr, pct)
            if ok:
                with self.lock:
                    ctrl.aplicar_trayectoria(tr)
        (self.get_logger().info if ok else self.get_logger().warning)(msg)
        return ok, msg

    def _cb_objetivo_deg(self, msg: Float64MultiArray) -> None:
        ctrl = self.ctrl
        if ctrl is None:
            return
        data = np.asarray(msg.data, dtype=float)
        n_total = len(self.cfg.articulaciones)
        n_act = ctrl.n
        pct = None
        if data.size in (n_total, n_total + 1):
            if data.size == n_total + 1:
                pct = data[-1]
            idx = [a.indice for a in ctrl.movibles]
            angulos = data[:n_total][idx]
        elif data.size in (n_act, n_act + 1):
            if data.size == n_act + 1:
                pct = data[-1]
            angulos = data[:n_act]
        else:
            self.get_logger().error(
                f'/brazo/objetivo_deg: se esperaban {n_total} valores (todas '
                f'las articulaciones del perfil) o {n_act} (sólo las activas: '
                f'{ctrl.nombres}), más un % opcional; llegaron {data.size}')
            return
        self._mover(np.radians(angulos), pct)

    def _cb_objetivo_nombres(self, msg: JointState) -> None:
        ctrl = self.ctrl
        if ctrl is None:
            return
        try:
            obj = ctrl.objetivo_por_nombres(
                {n: math.radians(p) for n, p in zip(msg.name, msg.position)})
        except KeyError as exc:
            self.get_logger().error(f'objetivo por nombres: {exc}')
            return
        pct = msg.velocity[0] if len(msg.velocity) > 0 else None
        self._mover(obj, pct)

    def _cb_objetivo_rad(self, msg: JointState) -> None:
        ctrl = self.ctrl
        if ctrl is None:
            return
        obj = np.full(ctrl.n, np.nan)
        for n, p in zip(msg.name, msg.position):
            if n in ctrl.nombres:
                obj[ctrl.nombres.index(n)] = p
        self._mover(obj, None)

    # ==================================================================
    # Paro, heartbeat, habilitar
    # ==================================================================
    def _hacer_paro(self, motivo: str) -> None:
        with self.lock:
            if self.ctrl is not None:
                self.ctrl.paro(motivo)
                if self.autotune and not self.autotune.terminado:
                    self.autotune.cancelar('paro de emergencia')
                if self.ctrl.paro_accion == 'sin_corriente':
                    self._publicar_cero()
        self._publicar_paro()

    def _cb_paro_topico(self, msg: Bool) -> None:
        if msg.data:
            self._hacer_paro('tópico /brazo/paro_emergencia')

    def _cb_heartbeat(self, _msg: Empty) -> None:
        self.t_heartbeat = time.monotonic()

    def _cb_imu(self, msg: Imu) -> None:
        a = msg.linear_acceleration
        st = msg.header.stamp
        self._imu.append((st.sec + st.nanosec * 1e-9,
                          math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)))

    def _srv_paro(self, _req, res):
        self._hacer_paro('servicio ~/paro_emergencia')
        res.success, res.message = True, 'Paro de emergencia activo'
        return res

    def _srv_liberar(self, _req, res):
        with self.lock:
            if self.ctrl is None:
                res.success, res.message = False, 'Nodo sin configurar'
                return res
            ok, msg = self.ctrl.liberar_paro()
        self._publicar_paro()
        res.success, res.message = ok, msg
        return res

    def _srv_habilitar(self, req, res):
        with self.lock:
            if self.ctrl is None or not self.activo:
                res.success, res.message = False, 'Nodo no activo'
                return res
            if req.data:
                ok, msg = self.ctrl.habilitar()
            else:
                self.ctrl.deshabilitar('servicio')
                self._publicar_cero()
                ok, msg = True, 'Deshabilitado'
        res.success, res.message = ok, msg
        return res

    def _srv_calibracion(self, req, res):
        self.modo_calibracion = bool(req.data)
        self.t_heartbeat = time.monotonic()
        res.success = True
        res.message = ('Modo calibración: se exige heartbeat cada '
                       f'{self.cfg.seguridad["heartbeat_timeout"]} s'
                       if req.data else 'Modo calibración desactivado')
        self.get_logger().info(res.message)
        return res

    # ==================================================================
    # Perfil y guardado
    # ==================================================================
    def _srv_cargar_perfil(self, req, res):
        from rclpy.parameter import Parameter
        if self.autotune and not self.autotune.terminado:
            res.exito, res.mensaje = False, 'Autotuning en curso'
            return res
        nuevos = [Parameter('montadas', Parameter.Type.INTEGER,
                            int(req.montadas)),
                  Parameter('estados', Parameter.Type.STRING,
                            str(req.estados))]
        if req.brazo:
            nuevos.append(Parameter('brazo', Parameter.Type.STRING,
                                    str(req.brazo)))
        anteriores = []
        for p in nuevos:
            actual = self.get_parameter(p.name).value
            anteriores.append(Parameter(p.name, p.type_, actual))
        self.set_parameters(nuevos)
        ok = self._reconfigurar()
        if not ok:
            self.set_parameters(anteriores)
            self._reconfigurar()
            res.exito = False
            res.mensaje = ('No se pudo cargar (ver el log del nodo); se '
                           'restauró la configuración anterior')
            return res
        res.exito = True
        res.mensaje = f'Cargado: {self.cfg.resumen()}'
        return res

    def _reconfigurar(self) -> bool:
        if self.activo:
            self._transicion('deactivate')
        if self.ctrl is not None:
            self._transicion('cleanup')
        self._transicion('configure')
        if self.ctrl is None:
            return False
        self._transicion('activate')
        return True

    def _yaml_ajustes(self, incluir_estados: bool) -> dict:
        c = self.ctrl
        arts: Dict[str, dict] = {}
        for g, k in enumerate(c.k_gpi):
            n = c.nombres[k]
            arts[n] = {
                'polos': [round(float(p), 4) for p in c.gpi.polos[g]],
                'beta': round(float(c.beta[g]), 4),
                'alfa': round(float(c.alfa[g]), 4),
                'armadura': round(float(c.armadura[g]), 4),
                'alfa_gravedad': round(float(c.alfa_g[g]), 4),
                'friccion_coulomb': round(float(c.fc[g]), 4),
                'limite_mando': round(float(c.limite[g]), 4),
            }
        if incluir_estados:
            for a in self.cfg.articulaciones:
                arts.setdefault(a.nombre, {})['estado'] = a.estado
        sh = c.cfg.shaper
        return {
            'articulaciones': arts,
            'trayectoria': {'velocidad_pct_defecto': float(c.pct_defecto)},
            'shaper': {'tipo': str(sh['tipo']),
                       'frecuencia_hz': float(sh['frecuencia_hz']),
                       'amortiguamiento': float(sh['amortiguamiento']),
                       'tolerancia_ei': float(sh['tolerancia_ei'])},
            'supervisor': {'habilitado': bool(c.supervisor.habilitado)},
        }

    def guardar(self, archivo: str = '', incluir_estados: bool = False):
        """Escribe el YAML de ajustes. Devuelve (ok, mensaje, ruta)."""
        if self.ctrl is None:
            return False, 'Nodo sin configurar', ''
        ruta = archivo or self.archivo_ajustes
        with self.lock:
            datos = self._yaml_ajustes(incluir_estados)
        try:
            os.makedirs(os.path.dirname(ruta), exist_ok=True)
            encabezado = (
                f'# Ajustes del brazo "{self.cfg.nombre}" (autotuning y '
                f'cambios guardados desde la interfaz).\n# Se aplican ENCIMA '
                f'del perfil y de la plataforma al arrancar. Editable a '
                f'mano.\n# Guardado: {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
            with open(ruta, 'w', encoding='utf-8') as fh:
                fh.write(encabezado)
                yaml.safe_dump(datos, fh, allow_unicode=True,
                               sort_keys=False, default_flow_style=None)
        except OSError as exc:
            return False, f'No se pudo escribir {ruta}: {exc}', ruta
        if '/install/' in ruta:
            aviso = (' (ojo: está en install/; compila con --symlink-install '
                     'para que se guarde en src/)')
        else:
            aviso = ''
        return True, f'Guardado en {ruta}{aviso}', ruta

    def _srv_guardar(self, req, res):
        ok, msg, ruta = self.guardar(req.archivo, req.incluir_estados)
        res.exito, res.mensaje, res.archivo = ok, msg, ruta
        (self.get_logger().info if ok else self.get_logger().error)(msg)
        return res

    def _srv_estimar_modo(self, _req, res):
        c = self.ctrl
        if c is None:
            res.success, res.message = False, 'Nodo sin configurar'
            return res
        with self.lock:
            reg = list(c.registro_phi)
            t_fin = c.t_fin_movimiento
        if len(reg) < 64:
            res.success, res.message = False, 'Muy pocos datos de phi_hat'
            return res
        t = np.array([r[0] for r in reg])
        phi = np.array([r[1] for r in reg])
        sel = t >= (t_fin if t_fin is not None else t[0])
        if sel.sum() < 64:
            sel = np.ones_like(t, dtype=bool)
        fs = 1.0 / max(np.median(np.diff(t)), 1e-4)
        lineas = []
        mejor = None
        for g, k in enumerate(c.k_gpi):
            r = estimar_modo(phi[sel, g], fs)
            if r is None:
                continue
            lineas.append(f'{c.nombres[k]}: f = {r["frecuencia_hz"]:.2f} Hz, '
                          f'ζ = {r["amortiguamiento"]:.3f} ({r["metodo"]})')
            if mejor is None or r['amplitud'] > mejor['amplitud']:
                mejor = r
        if len(self._imu) > 64:
            ti = np.array([x[0] for x in self._imu])
            ai = np.array([x[1] for x in self._imu])
            fsi = 1.0 / max(np.median(np.diff(ti)), 1e-4)
            r = estimar_modo(ai, fsi)
            if r is not None:
                lineas.append(f'IMU: f = {r["frecuencia_hz"]:.2f} Hz, ζ = '
                              f'{r["amortiguamiento"]:.3f}')
        if mejor is None:
            res.success, res.message = False, 'No se encontró un pico claro'
            return res
        lineas.append(f'Sugerencia: shaper.frecuencia_hz = '
                      f'{mejor["frecuencia_hz"]:.2f}, shaper.amortiguamiento '
                      f'= {max(mejor["amortiguamiento"], 0.0):.3f}')
        res.success, res.message = True, '\n'.join(lineas)
        return res

    # ==================================================================
    # Acciones
    # ==================================================================
    def _goal_ok(self, _goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_ok(self, _goal) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _exec_mover(self, gh):
        req = gh.request
        res = MoverA.Result()
        ctrl = self.ctrl
        if ctrl is None:
            res.exito, res.mensaje = False, 'Nodo sin configurar'
            gh.abort()
            return res
        try:
            if req.nombres:
                obj = ctrl.objetivo_por_nombres(
                    {n: math.radians(p) for n, p in
                     zip(req.nombres, req.posiciones_deg)})
            else:
                if len(req.posiciones_deg) != ctrl.n:
                    raise KeyError(f'se esperaban {ctrl.n} posiciones '
                                   f'({ctrl.nombres})')
                obj = np.radians(np.asarray(req.posiciones_deg, float))
        except KeyError as exc:
            res.exito, res.mensaje = False, str(exc)
            gh.abort()
            return res
        pct = req.velocidad_pct if req.velocidad_pct > 0.0 else None
        ok, msg = self._mover(obj, pct)
        if not ok:
            res.exito, res.mensaje = False, msg
            gh.abort()
            return res
        t_inicio = time.monotonic()
        fb = MoverA.Feedback()
        while True:
            time.sleep(0.05)
            with self.lock:
                estado = ctrl.estado
                tr = ctrl.traj
                prog = (min(1.0, (ctrl.t_prev - tr.t0) / tr.duration)
                        if tr.duration > 0 and ctrl.t_prev else 1.0)
                err = np.degrees(ctrl.q - ctrl.q_d)
                terminado = ctrl.movimiento_terminado()
            if gh.is_cancel_requested:
                with self.lock:
                    ctrl.traj.fijar(ctrl.q_d)
                    ctrl.objetivo = ctrl.q_d.copy()
                gh.canceled()
                res.exito, res.mensaje = False, 'Cancelado (se queda donde va)'
                return res
            if estado != 'habilitado':
                gh.abort()
                res.exito, res.mensaje = False, f'Control {estado}'
                return res
            fb.progreso = float(prog)
            fb.error_deg = [float(e) for e in err]
            gh.publish_feedback(fb)
            if terminado:
                break
            if time.monotonic() - t_inicio > 600.0:
                gh.abort()
                res.exito, res.mensaje = False, 'Tiempo de espera agotado'
                return res
        # espera la ventana del supervisor para reportar el sobreimpulso
        time.sleep(float(self.cfg.supervisor['ventana']) + 0.1)
        with self.lock:
            res.error_final_deg = [float(e) for e in
                                   np.degrees(ctrl.q - ctrl.q_d)]
            res.sobreimpulso_deg = [float(s) for s in
                                    np.degrees(ctrl.sobreimpulso)]
        res.exito, res.mensaje = True, msg
        gh.succeed()
        return res

    def _exec_autotune(self, gh):
        req = gh.request
        res = AutotuneAccion.Result()
        ctrl = self.ctrl
        try:
            with self.lock:
                if ctrl is None:
                    raise ValueError('Nodo sin configurar')
                if self.autotune and not self.autotune.terminado:
                    raise ValueError('Ya hay un autotuning en curso')
                at = Autotune(ctrl, list(req.articulaciones) or None,
                              log=self.get_logger().info)
                at.iniciar()
                self.autotune = at
        except ValueError as exc:
            res.exito, res.mensaje = False, str(exc)
            gh.abort()
            return res
        fb = AutotuneAccion.Feedback()
        while not at.terminado:
            time.sleep(0.2)
            if gh.is_cancel_requested:
                with self.lock:
                    at.cancelar('cancelado por el usuario')
                gh.canceled()
                res.exito, res.mensaje = False, 'Cancelado; ganancias restauradas'
                return res
            fb.progreso = float(at.progreso)
            fb.etapa = at.etapa
            gh.publish_feedback(fb)
        res.resultados_yaml = yaml.safe_dump(at.resultados, allow_unicode=True,
                                             sort_keys=False)
        if not at.exito:
            res.exito, res.mensaje = False, at.mensaje
            gh.abort()
            return res
        mensaje = at.mensaje
        if not req.aplicar:
            with self.lock:
                at._terminar(True, at.mensaje, restaurar=True)
            mensaje += ' (resultados NO aplicados)'
        elif req.guardar:
            ok, msg_g, _ = self.guardar()
            mensaje += f'; {msg_g}'
        self._refrescar_parametros()
        res.exito, res.mensaje = True, mensaje
        gh.succeed()
        return res

    def _refrescar_parametros(self) -> None:
        """Copia a los parámetros ROS los valores actuales del control."""
        from rclpy.parameter import Parameter
        c = self.ctrl
        params = []
        for g, k in enumerate(c.k_gpi):
            n = c.nombres[k]
            params.append(Parameter(f'{n}.polos',
                                    Parameter.Type.DOUBLE_ARRAY,
                                    [float(p) for p in c.gpi.polos[g]]))
            for campo, arr in (('beta', c.beta), ('alfa', c.alfa),
                               ('armadura', c.armadura),
                               ('alfa_gravedad', c.alfa_g),
                               ('friccion_coulomb', c.fc)):
                params.append(Parameter(f'{n}.{campo}',
                                        Parameter.Type.DOUBLE,
                                        float(arr[g])))
        self._cargando_params = True
        try:
            self.set_parameters(params)
        finally:
            self._cargando_params = False


def main(args=None) -> None:
    """Punto de entrada: crea el nodo, autoarranca y gira el executor."""
    rclpy.init(args=args)
    nodo = NodoControl()
    if nodo.get_parameter('autoarranque').value:
        nodo.arrancar()
    # lazo + watchdog + acciones largas (mover, autotune) + servicios
    ejecutor = MultiThreadedExecutor(num_threads=8)
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            with nodo.lock:
                if nodo.ctrl is not None:
                    nodo._publicar_cero()
        except Exception:  # noqa: BLE001
            pass
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
