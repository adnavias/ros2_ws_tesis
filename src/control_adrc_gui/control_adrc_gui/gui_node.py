"""Nodo de la interfaz web del brazo ("control_gui").

Sirve una página web (servidor.py, sólo biblioteca estándar) y la conecta
con el control usando ÚNICAMENTE sus interfaces públicas de ROS 2:

    suscribe   /adrc_simple/estado            EstadoControl
               /brazo/paro_emergencia/estado  Bool (latched)
    publica    /brazo/paro_emergencia          Bool (respaldo del servicio)
               /brazo/interfaz/heartbeat       Empty (sólo si hay navegador)
    servicios  /adrc_simple/{habilitar, paro_emergencia, liberar_paro,
               modo_calibracion, cargar_perfil, guardar_configuracion,
               estimar_modo, set_parameters}
    acciones   /adrc_simple/{mover_a, autotune}

No hace falta instalar nada: corre en la Jetson (o en WSL) y se abre desde
cualquier navegador de la red del rover en http://<IP>:<puerto>.

Heartbeat. La página manda un latido cada 0.2 s; el nodo sólo publica el
heartbeat ROS mientras el último latido del navegador tenga menos de
`timeout_navegador` s. Si se cierra la pestaña o se cae la red, el
heartbeat se detiene y, en modo calibración, el control pasa a paro.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import os
import socket
import subprocess
import threading
import time
import webbrowser
from typing import List, Optional

import rclpy
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty
from std_srvs.srv import SetBool, Trigger

from control_adrc_gui.servidor import ServidorWeb
from control_adrc_interfaces.action import Autotune, MoverA
from control_adrc_interfaces.msg import EstadoControl
from control_adrc_interfaces.srv import CargarPerfil, GuardarConfiguracion


def _es_wsl() -> bool:
    try:
        with open('/proc/version', 'r', encoding='utf-8') as fh:
            return 'microsoft' in fh.read().lower()
    except OSError:
        return False


def _ips_locales() -> List[str]:
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return ips or ['127.0.0.1']


class NodoGui(Node):
    """Puente entre la página web y las interfaces del control."""

    def __init__(self) -> None:
        """Parámetros, clientes y servidor web."""
        super().__init__('control_gui')
        self.declare_parameter('nodo_control', '/adrc_simple')
        self.declare_parameter('puerto', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('abrir_navegador', False)
        self.declare_parameter('timeout_navegador', 0.6)
        self.declare_parameter('directorio_brazos', '')
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        ctrl = str(gp('nodo_control')).rstrip('/')
        self.ctrl = ctrl
        self.timeout_nav = float(gp('timeout_navegador'))
        cb = ReentrantCallbackGroup()
        self.cb = cb

        self.lock = threading.Lock()
        self.ultimo_estado: Optional[EstadoControl] = None
        self.t_estado = 0.0
        self.paro_latched = False
        self.t_latido = 0.0
        self.autotune = {'activo': False, 'progreso': 0.0, 'etapa': '',
                         'resultado': '', 'mensaje': ''}
        self.movimiento = {'activo': False, 'mensaje': ''}
        self._goal_autotune = None
        self._goal_mover = None

        self.create_subscription(EstadoControl, f'{ctrl}/estado',
                                 self._cb_estado, 10, callback_group=cb)
        qos_latch = QoSProfile(depth=1,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/brazo/paro_emergencia/estado',
                                 self._cb_paro, qos_latch, callback_group=cb)
        self.pub_paro = self.create_publisher(Bool, '/brazo/paro_emergencia',
                                              10)
        self.pub_hb = self.create_publisher(Empty,
                                            '/brazo/interfaz/heartbeat', 10)
        self.srv = {
            'habilitar': self.create_client(SetBool, f'{ctrl}/habilitar',
                                            callback_group=cb),
            'paro': self.create_client(Trigger, f'{ctrl}/paro_emergencia',
                                       callback_group=cb),
            'liberar': self.create_client(Trigger, f'{ctrl}/liberar_paro',
                                          callback_group=cb),
            'calibracion': self.create_client(
                SetBool, f'{ctrl}/modo_calibracion', callback_group=cb),
            'perfil': self.create_client(CargarPerfil,
                                         f'{ctrl}/cargar_perfil',
                                         callback_group=cb),
            'guardar': self.create_client(GuardarConfiguracion,
                                          f'{ctrl}/guardar_configuracion',
                                          callback_group=cb),
            'modo': self.create_client(Trigger, f'{ctrl}/estimar_modo',
                                       callback_group=cb),
            'parametros': self.create_client(SetParameters,
                                             f'{ctrl}/set_parameters',
                                             callback_group=cb),
        }
        self.acc_mover = ActionClient(self, MoverA, f'{ctrl}/mover_a',
                                      callback_group=cb)
        self.acc_autotune = ActionClient(self, Autotune, f'{ctrl}/autotune',
                                         callback_group=cb)
        self.create_timer(0.1, self._cb_heartbeat, callback_group=cb)

        self.web = ServidorWeb(self, str(gp('host')), int(gp('puerto')))
        self.web.iniciar()
        url_local = f'http://localhost:{self.web.puerto}'
        urls = [f'http://{ip}:{self.web.puerto}' for ip in _ips_locales()]
        self.get_logger().info(
            f'Interfaz web lista: {url_local} (en esta máquina) | desde otra '
            f'computadora de la red: {", ".join(urls)}')
        if gp('abrir_navegador'):
            threading.Thread(target=self._abrir, args=(url_local,),
                             daemon=True).start()

    # ------------------------------------------------------------------
    def _abrir(self, url: str) -> None:
        time.sleep(1.0)
        try:
            if _es_wsl():
                subprocess.Popen(['explorer.exe', url],
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
            else:
                webbrowser.open(url)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f'No pude abrir el navegador ({exc}); abre {url} a mano')

    def _cb_estado(self, msg: EstadoControl) -> None:
        with self.lock:
            self.ultimo_estado = msg
            self.t_estado = time.monotonic()

    def _cb_paro(self, msg: Bool) -> None:
        self.paro_latched = bool(msg.data)

    def _cb_heartbeat(self) -> None:
        if time.monotonic() - self.t_latido < self.timeout_nav:
            self.pub_hb.publish(Empty())

    # ------------------------------------------------------------------
    # Estado para la página
    # ------------------------------------------------------------------
    def estado(self) -> dict:
        """Snapshot JSON del control (lo llama el servidor web)."""
        with self.lock:
            m = self.ultimo_estado
            edad = time.monotonic() - self.t_estado
        base = {'conectado': m is not None and edad < 1.0,
                'edad_estado': edad if m is not None else None,
                'paro_latched': self.paro_latched,
                'autotune': dict(self.autotune),
                'movimiento': dict(self.movimiento),
                'perfiles': self._perfiles(),
                't': time.time()}
        if m is None:
            return base
        n = len(m.nombres)
        base.update({
            'brazo': m.brazo, 'plataforma': m.plataforma,
            'estado': m.estado, 'motivo': m.motivo, 'watchdog': m.watchdog,
            'paro_emergencia': m.paro_emergencia,
            'modo_calibracion': m.modo_calibracion,
            'trayectoria_activa': m.trayectoria_activa,
            'autotune_activo': m.autotune_activo,
            'autotune_progreso': m.autotune_progreso,
            'autotune_etapa': m.autotune_etapa,
            'rampa_gravedad': m.rampa_gravedad,
            'nombres_perfil': list(m.nombres_perfil),
            'estados_perfil': list(m.estados_perfil),
            'nombres': list(m.nombres), 'mando': list(m.mando),
            'posicion_deg': list(m.posicion_deg),
            'referencia_deg': list(m.referencia_deg),
            'error_deg': list(m.error_deg),
            'mando_valor': list(m.mando_valor),
            'limite_mando': list(m.limite_mando),
            'phi_hat': list(m.phi_hat), 'kappa': list(m.kappa),
            'wn': list(m.wn),
            'polos': [list(m.polos[4 * i:4 * i + 4]) for i in range(n)],
            'beta': list(m.beta), 'alfa': list(m.alfa),
            'armadura': list(m.armadura),
            'alfa_gravedad': list(m.alfa_gravedad),
            'friccion_coulomb': list(m.friccion_coulomb),
            'supervisor_habilitado': m.supervisor_habilitado,
            'sobreimpulso_deg': list(m.sobreimpulso_deg),
            'saturaciones': list(m.saturaciones),
            'saturado': list(m.saturado),
            'velocidad_pct_defecto': m.velocidad_pct_defecto,
            'shaper': {'tipo': m.shaper_tipo,
                       'frecuencia_hz': m.shaper_frecuencia_hz,
                       'amortiguamiento': m.shaper_amortiguamiento},
            'eventos': list(m.eventos),
        })
        return base

    def _perfiles(self) -> List[str]:
        directorio = self.get_parameter('directorio_brazos').value
        if not directorio:
            try:
                from control_adrc_rover.rutas import dir_config
                directorio = os.path.join(dir_config(), 'brazos')
            except Exception:  # noqa: BLE001
                return []
        try:
            return sorted(f[:-5] for f in os.listdir(directorio)
                          if f.endswith('.yaml'))
        except OSError:
            return []

    # ------------------------------------------------------------------
    # Comandos desde la página
    # ------------------------------------------------------------------
    def _llamar(self, nombre: str, req, timeout: float = 5.0):
        cli = self.srv[nombre]
        if not cli.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f'El servicio {cli.srv_name} no responde; ¿está '
                               f'corriendo el control?')
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f'{cli.srv_name}: sin respuesta en '
                                   f'{timeout:.0f} s')
            time.sleep(0.01)
        return fut.result()

    def comando(self, accion: str, d: dict):
        """Ejecuta una acción de la página. Devuelve (ok, mensaje, extra)."""
        if accion == 'heartbeat':
            self.t_latido = time.monotonic()
            return True, '', {}
        if accion == 'paro':
            self.pub_paro.publish(Bool(data=True))     # vía rápida
            try:
                r = self._llamar('paro', Trigger.Request(), 2.0)
                return r.success, r.message, {}
            except RuntimeError as exc:
                return True, f'Paro enviado por tópico ({exc})', {}
        if accion == 'liberar':
            r = self._llamar('liberar', Trigger.Request())
            return r.success, r.message, {}
        if accion in ('habilitar', 'deshabilitar'):
            r = self._llamar('habilitar',
                             SetBool.Request(data=accion == 'habilitar'))
            return r.success, r.message, {}
        if accion == 'calibracion':
            self.t_latido = time.monotonic()
            r = self._llamar('calibracion',
                             SetBool.Request(data=bool(d.get('activar'))))
            return r.success, r.message, {}
        if accion == 'perfil':
            req = CargarPerfil.Request(brazo=str(d.get('brazo', '')),
                                       montadas=int(d.get('montadas', 0)),
                                       estados=str(d.get('estados', '')))
            r = self._llamar('perfil', req, 20.0)
            return r.exito, r.mensaje, {}
        if accion == 'guardar':
            req = GuardarConfiguracion.Request(
                archivo='', incluir_estados=bool(d.get('incluir_estados')))
            r = self._llamar('guardar', req)
            return r.exito, r.mensaje, {'archivo': r.archivo}
        if accion == 'estimar_modo':
            r = self._llamar('modo', Trigger.Request())
            return r.success, r.message, {}
        if accion == 'parametros':
            return self._parametros(d.get('parametros') or {})
        if accion == 'mover':
            return self._mover(d)
        if accion == 'autotune':
            return self._autotune(d)
        if accion == 'cancelar_autotune':
            gh = self._goal_autotune
            if gh is None:
                return False, 'No hay autotuning en curso', {}
            gh.cancel_goal_async()
            return True, 'Cancelación pedida', {}
        if accion == 'cancelar_movimiento':
            gh = self._goal_mover
            if gh is None:
                return False, 'No hay movimiento en curso', {}
            gh.cancel_goal_async()
            return True, 'Cancelación pedida', {}
        return False, f'Acción desconocida: {accion}', {}

    def _parametros(self, params: dict):
        req = SetParameters.Request()
        for nombre, valor in params.items():
            v = ParameterValue()
            if isinstance(valor, bool):
                v.type, v.bool_value = ParameterType.PARAMETER_BOOL, valor
            elif isinstance(valor, (int, float)):
                v.type = ParameterType.PARAMETER_DOUBLE
                v.double_value = float(valor)
            elif isinstance(valor, list):
                v.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                v.double_array_value = [float(x) for x in valor]
            else:
                v.type, v.string_value = (ParameterType.PARAMETER_STRING,
                                          str(valor))
            req.parameters.append(ParamMsg(name=str(nombre), value=v))
        r = self._llamar('parametros', req)
        errores = [f'{p.name}: {res.reason}' for p, res in
                   zip(req.parameters, r.results) if not res.successful]
        if errores:
            return False, 'Rechazado: ' + '; '.join(errores), {}
        return True, f'Aplicado: {", ".join(params)}', {}

    def _mover(self, d: dict):
        if not self.acc_mover.wait_for_server(timeout_sec=1.0):
            return False, 'La acción mover_a no responde', {}
        goal = MoverA.Goal()
        posiciones = d.get('posiciones_deg') or {}
        goal.nombres = list(posiciones.keys())
        goal.posiciones_deg = [float('nan') if v is None else float(v)
                               for v in posiciones.values()]
        goal.velocidad_pct = float(d.get('velocidad_pct') or 0.0)
        fut = self.acc_mover.send_goal_async(goal)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > 3.0:
                return False, 'Sin respuesta de mover_a', {}
            time.sleep(0.01)
        gh = fut.result()
        if not gh.accepted:
            return False, 'Objetivo rechazado', {}
        self._goal_mover = gh
        self.movimiento = {'activo': True, 'mensaje': 'en movimiento'}

        def _fin(f):
            r = f.result().result
            self.movimiento = {'activo': False, 'mensaje': r.mensaje,
                               'sobreimpulso_deg': list(r.sobreimpulso_deg),
                               'error_final_deg': list(r.error_final_deg),
                               'exito': r.exito}
            self._goal_mover = None
        gh.get_result_async().add_done_callback(_fin)
        return True, 'Objetivo enviado', {}

    def _autotune(self, d: dict):
        if not self.acc_autotune.wait_for_server(timeout_sec=1.0):
            return False, 'La acción autotune no responde', {}
        goal = Autotune.Goal()
        goal.articulaciones = [str(n) for n in d.get('articulaciones') or []]
        goal.aplicar = bool(d.get('aplicar', True))
        goal.guardar = bool(d.get('guardar', False))

        def _fb(msg):
            self.autotune.update({'activo': True,
                                  'progreso': msg.feedback.progreso,
                                  'etapa': msg.feedback.etapa})
        fut = self.acc_autotune.send_goal_async(goal, feedback_callback=_fb)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > 3.0:
                return False, 'Sin respuesta de autotune', {}
            time.sleep(0.01)
        gh = fut.result()
        if not gh.accepted:
            return False, 'Autotuning rechazado', {}
        self._goal_autotune = gh
        self.autotune = {'activo': True, 'progreso': 0.0,
                         'etapa': 'iniciando', 'resultado': '',
                         'mensaje': ''}

        def _fin(f):
            r = f.result().result
            self.autotune = {'activo': False, 'progreso': 1.0 if r.exito
                             else self.autotune.get('progreso', 0.0),
                             'etapa': 'terminado' if r.exito else 'abortado',
                             'resultado': r.resultados_yaml,
                             'mensaje': r.mensaje, 'exito': r.exito}
            self._goal_autotune = None
        gh.get_result_async().add_done_callback(_fin)
        return True, 'Autotuning iniciado', {}


def main(args=None) -> None:
    """Punto de entrada."""
    rclpy.init(args=args)
    nodo = NodoGui()
    ejecutor = MultiThreadedExecutor(num_threads=4)
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.web.detener()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
