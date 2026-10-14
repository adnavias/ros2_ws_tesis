"""Controlador del brazo, independiente de ROS.

Une todas las piezas del núcleo en un lazo que se ejecuta en cada
medición nueva:

    medición (raw) -> sanidad -> q_urdf
    trayectoria (q*, q*', q*'') -> conformador -> referencia
    GPI (v) + paredes virtuales
    inversa de B:  i = K^-1 ( M_ctrl(q) v + r(t) g_cal(q) ) + comp. fricción
    saturación -> límite de pendiente -> anti-windup -> mando

Estados:
    esperando_medicion  aún no llega la posición de todas las activas.
    deshabilitado       hay medición, no se manda corriente.
    habilitado          lazo cerrado.
    paro                paro de emergencia (latched): corriente cero o
                        congelar, según `seguridad.paro_accion`. Liberar lo
                        deja en "deshabilitado".
    falla               medición perdida (> watchdog_quitar), encoder
                        inválido o error excesivo: corriente cero. Se sale
                        con habilitar() cuando vuelve la medición.

Tiempo. El tiempo del controlador es el de la MEDICIÓN (header.stamp del
JointState; sim time en Gazebo, reloj del robot en la Jetson). Así la
trayectoria y el dt de integración siguen a la planta aunque Gazebo corra
más lento que el tiempo real. El watchdog, en cambio, usa la edad de la
última medición medida con un reloj monótono (lo calcula el nodo).

Coordenadas. Internamente todo está en convención URDF (modelo, límites).
Los objetivos del usuario son relativos a la postura de arranque si
`lazo.referencia_objetivos = relativa_arranque` (comportamiento histórico)
o absolutos en URDF si vale `urdf`.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from .config import ConfigBrazo
from .gpi import GPI
from .modelo import ModeloBrazo, calibrar, escalar_beta
from .seguridad import (EstimadorVelocidad, LimitadorPendiente,
                        SanidadEncoder, aceleracion_pared, validar_objetivo)
from .shaper import ReferenciaConformada, duracion_minima_modo
from .supervisor import Supervisor, analizar, estimar_kappa_lote
from .trayectoria import TrayectoriaNonica

ESTADOS_CONTROL = ('esperando_medicion', 'deshabilitado', 'habilitado',
                   'paro', 'falla')


@dataclass
class Mando:
    """Salida de un paso del controlador (una entrada por articulación
    movible, en el orden de `Controlador.movibles`)."""

    valores: np.ndarray            # en unidades del actuador, con signo
    publicar: np.ndarray           # máscara: qué articulaciones publicar
    tipos: List[str]


@dataclass
class Diagnostico:
    """Estado interno para publicar (unidades del usuario: rad)."""

    estado: str = 'esperando_medicion'
    watchdog: str = 'ok'
    q: np.ndarray = field(default_factory=lambda: np.zeros(0))
    q_ref: np.ndarray = field(default_factory=lambda: np.zeros(0))
    error: np.ndarray = field(default_factory=lambda: np.zeros(0))
    mando: np.ndarray = field(default_factory=lambda: np.zeros(0))
    phi_hat: np.ndarray = field(default_factory=lambda: np.zeros(0))
    kappa: np.ndarray = field(default_factory=lambda: np.zeros(0))
    saturaciones: np.ndarray = field(default_factory=lambda: np.zeros(0))
    saturado: np.ndarray = field(default_factory=lambda: np.zeros(0))
    sobreimpulso: np.ndarray = field(default_factory=lambda: np.zeros(0))
    wn: np.ndarray = field(default_factory=lambda: np.zeros(0))
    beta: np.ndarray = field(default_factory=lambda: np.zeros(0))
    trayectoria_activa: bool = False
    rampa_gravedad: float = 0.0
    dt: float = 0.0


class Controlador:
    """Lazo completo del brazo para n articulaciones (sin ROS)."""

    def __init__(self, cfg: ConfigBrazo, modelo: Optional[ModeloBrazo],
                 log: Callable[[str], None] = print,
                 aviso: Callable[[str], None] = None) -> None:
        """Prepara índices, ganancias, trayectoria y protecciones."""
        self.cfg = cfg
        self.modelo = modelo if (modelo is not None and modelo.n > 0) else None
        self.log = log
        self.aviso = aviso or log
        lazo = cfg.lazo
        self.dt_nominal = 1.0 / float(lazo['frecuencia_control'])

        # --- Conjuntos de articulaciones ---
        self.movibles = cfg.movibles                    # activas
        self.n = len(self.movibles)
        self.nombres = [a.nombre for a in self.movibles]
        self.k_gpi = np.array([i for i, a in enumerate(self.movibles)
                               if a.usa_gpi], dtype=int)
        self.k_pos = np.array([i for i, a in enumerate(self.movibles)
                               if a.mando == 'posicion_directa'], dtype=int)
        self.k_med = np.array([i for i, a in enumerate(self.movibles)
                               if a.medida], dtype=int)
        gpi_arts = [self.movibles[i] for i in self.k_gpi]
        # dentro de los GPI: cuáles usan el modelo (orden = modelo.nombres)
        if self.modelo is not None:
            self.g_mod = np.array([self.nombres_gpi_index(nom)
                                   for nom in self.modelo.nombres], dtype=int)
        else:
            self.g_mod = np.zeros(0, dtype=int)
            if any(a.en_modelo for a in self.movibles):
                self.aviso('Sin modelo dinámico: las articulaciones con '
                           '"corriente_con_modelo" usan inercia_constante y '
                           'sin gravedad (peor desempeño).')
        self.g_sin = np.array([g for g in range(len(gpi_arts))
                               if g not in set(self.g_mod.tolist())],
                              dtype=int)

        # --- Parámetros por articulación movible ---
        f = lambda attr: np.array([getattr(a, attr) for a in self.movibles],  # noqa: E731
                                  dtype=float)
        self.signo = f('signo')
        self.offset = np.array([
            a.postura_encendido_urdf if a.encoder == 'incremental'
            else a.offset_absoluto for a in self.movibles])
        self.lo = f('limite_inferior')
        self.hi = f('limite_superior')
        self.margen = f('margen_limite')
        self.k_pared = f('rigidez_pared')
        self.d_pared = f('amortiguamiento_pared')
        self.unidad_deg = np.array([a.unidad_posicion == 'deg'
                                    for a in self.movibles])
        # GPI (vectores sobre k_gpi)
        self.k_tau = np.array([a.k_tau for a in gpi_arts])
        self.limite = np.array([a.limite_mando for a in gpi_arts])
        self.factor_limite = np.ones(len(gpi_arts))
        self.fc = np.array([a.friccion_coulomb for a in gpi_arts])
        self.fc_gain = np.array([4.0 / max(a.ancho_friccion, 1e-6)
                                 for a in gpi_arts])
        self.j_const = np.array([a.inercia_constante for a in gpi_arts])
        self.b_visc = np.array([a.amortiguamiento_viscoso for a in gpi_arts])
        self.beta = np.array([a.beta for a in gpi_arts])
        self.alfa = np.array([a.alfa for a in gpi_arts])
        self.armadura = np.array([
            a.armadura if lazo['incluir_armadura'] else 0.0
            for a in gpi_arts])
        self.alfa_g = np.array([a.alfa_gravedad for a in gpi_arts])
        self.es_corriente = np.array([a.mando == 'corriente_con_modelo'
                                      for a in gpi_arts])
        self.gpi = GPI(np.array([a.polos for a in gpi_arts]).reshape(-1, 4),
                       float(lazo['antiwindup_ganancia']))
        self.wn_inicial = self.gpi.wn().copy()
        self.pendiente = LimitadorPendiente([a.pendiente_max
                                             for a in gpi_arts])
        self.friccion_velocidad = (lazo.get('modelo_friccion', 'zona_muerta')
                                   == 'velocidad')

        # --- Trayectoria y conformador ---
        tr = cfg.trayectoria
        self.factor_100 = float(tr['factor_100pct'])
        self.pct_max = 100.0 / self.factor_100
        self.pct_min = float(tr['velocidad_pct_min'])
        self.pct_defecto = float(np.clip(tr['velocidad_pct_defecto'],
                                         self.pct_min, self.pct_max))
        self.traj = TrayectoriaNonica(
            self.factor_100 * f('v_max_fisica'), f('a_max_100pct'),
            float(tr['duracion_minima']), f('jerk_max_100pct'))
        sh = cfg.shaper
        self.shaper = ReferenciaConformada(sh['tipo'], sh['frecuencia_hz'],
                                           sh['amortiguamiento'],
                                           sh['tolerancia_ei'])

        # --- Protecciones ---
        seg = cfg.seguridad
        desenv = np.array([a.continua if a.desenvolver is None
                           else a.desenvolver for a in self.movibles])
        self.sanidad = SanidadEncoder(f('salto_max'), desenv,
                                      int(seg['muestras_invalidas_max']))
        self.vel = EstimadorVelocidad(self.n)
        self.error_max = math.radians(float(seg['error_max_deg']))
        self.accion_error = seg.get('accion_error', 'sostener')
        self.gracia_aborto = float(seg.get('gracia_aborto', 1.5))
        self._t_aborto: Optional[float] = None
        self.paro_accion = seg['paro_accion']

        # --- Supervisor, kappa y registro de phi_hat ---
        self.supervisor = Supervisor(cfg.supervisor)
        self.kappa = np.full(len(gpi_arts), np.nan)
        self.registro_phi: deque = deque(maxlen=int(20.0 / self.dt_nominal))
        self.t_fin_movimiento: Optional[float] = None

        # --- Estado ---
        self.estado = 'esperando_medicion'
        self.motivo = ''
        self.watchdog = 'ok'
        self.q = np.array([a.posicion_bloqueo for a in self.movibles])
        self.cero = self.q.copy()
        self.cero_fijado = False
        self.medido = np.zeros(self.n, dtype=bool)
        self.t_med: Optional[float] = None
        self.t_prev: Optional[float] = None
        self.objetivo = self.q.copy()
        self.q_d = self.q.copy()
        self.qd_d = np.zeros(self.n)
        self.qdd_d = np.zeros(self.n)
        self.r_grav = 0.0
        self.saturaciones = np.zeros(len(gpi_arts), dtype=int)
        self.saturado = np.zeros(len(gpi_arts), dtype=bool)
        self.sobreimpulso = np.zeros(self.n)
        self.pos_publicar = np.zeros(self.n, dtype=bool)
        self.ultimo_mando = np.zeros(self.n)
        self._mov: Optional[dict] = None
        self.observador: Optional[Callable[[dict], None]] = None
        self.data_ctrl = self.modelo.nuevo_data() if self.modelo else None
        self.data_plan = self.modelo.nuevo_data() if self.modelo else None
        self.diag = Diagnostico()
        self.eventos: List[str] = []

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------
    def nombres_gpi_index(self, nombre: str) -> int:
        """Índice dentro del vector GPI de la articulación `nombre`."""
        for g, k in enumerate(self.k_gpi):
            if self.movibles[k].nombre == nombre:
                return g
        raise KeyError(nombre)

    def _evento(self, texto: str) -> None:
        self.eventos.append(texto)
        self.log(texto)

    def a_usuario(self, q_urdf: np.ndarray) -> np.ndarray:
        """URDF -> coordenadas del usuario."""
        if self.cfg.lazo['referencia_objetivos'] == 'relativa_arranque':
            return q_urdf - self.cero
        return q_urdf

    def desde_usuario(self, q_usr: np.ndarray) -> np.ndarray:
        """Coordenadas del usuario -> URDF."""
        if self.cfg.lazo['referencia_objetivos'] == 'relativa_arranque':
            return q_usr + self.cero
        return q_usr

    def _modelo_cal(self, q: np.ndarray, data):
        """(M_cal, g_cal) en las articulaciones con modelo."""
        q_mod = q[self.k_gpi[self.g_mod]]
        m, g = self.modelo.calcular(q_mod, data)
        sel = self.g_mod
        return calibrar(m, g, self.alfa[sel], self.armadura[sel],
                        self.alfa_g[sel])

    # ------------------------------------------------------------------
    # Medición
    # ------------------------------------------------------------------
    def medir(self, q_raw: np.ndarray, t: float) -> bool:
        """Ingresa la medición de las articulaciones medidas (orden
        `k_med`) en el tiempo t. Devuelve True si hay que ejecutar paso()."""
        q_raw = np.asarray(q_raw, dtype=float)
        q_urdf_med = self.signo[self.k_med] * q_raw + self.offset[self.k_med]
        q_full = self.q.copy()
        q_full[self.k_med] = q_urdf_med
        q_ok, valida, falla = self.sanidad.procesar(q_full)
        # las no medidas (posición directa sin encoder) siguen la referencia
        sin_med = np.ones(self.n, dtype=bool)
        sin_med[self.k_med] = False
        self.q = np.where(sin_med, self.q_d, q_ok)
        if falla and self.estado == 'habilitado':
            self._falla('encoder: demasiadas muestras inválidas seguidas '
                        f'(rechazos {self.sanidad.rechazos.tolist()})')
        if not self.cero_fijado:
            self.cero = self.q.copy()
            self.cero_fijado = True
            self.objetivo = self.q.copy()
            self.q_d = self.q.copy()
            self._evento('Cero fijado en la postura actual: '
                         f'{np.round(self.q, 4).tolist()} rad (URDF)')
            self.estado = 'deshabilitado'
            if self.cfg.lazo['habilitar_al_arrancar']:
                self.habilitar()
        if self.t_med is not None and t <= self.t_med:
            return False                     # repetida o desordenada
        if self.t_med is not None:
            # decimación si la medición llega más rápido que el lazo
            if t - (self.t_prev if self.t_prev is not None else -1e9) < (
                    0.9 * self.dt_nominal):
                self.t_med = t
                return False
        self.t_med = t
        return True

    # ------------------------------------------------------------------
    # Estados
    # ------------------------------------------------------------------
    def habilitar(self) -> Tuple[bool, str]:
        """Cierra el lazo desde la postura actual (arranque suave)."""
        if self.estado == 'paro':
            return False, 'Paro de emergencia activo: libéralo primero'
        if not self.cero_fijado:
            return False, 'Aún no hay medición del brazo'
        if self.watchdog == 'quitar':
            return False, 'La medición no está llegando'
        self.objetivo = self.q.copy()
        self.q_d = self.q.copy()
        self.qd_d[:] = 0.0
        self.qdd_d[:] = 0.0
        self.traj.fijar(self.q)
        self.shaper.reiniciar()
        self.sanidad.reiniciar()
        self.vel.reiniciar()
        self.pendiente.reiniciar()
        self._mov = None
        # integradores: la primera corriente es la de gravedad
        self.gpi.reiniciar(self._v_gravedad())
        rampa = float(self.cfg.lazo['rampa_gravedad'])
        self.r_grav = 0.0 if (rampa > 0.0 and
                              self.cfg.lazo['feedforward_gravedad']) else (
            1.0 if self.cfg.lazo['feedforward_gravedad'] else 0.0)
        if self.r_grav == 1.0:
            self.gpi.reiniciar(None)        # la gravedad va completa por ff
        self.t_prev = None
        self.estado = 'habilitado'
        self.motivo = ''
        self._evento('Control habilitado')
        return True, 'Habilitado'

    def _v_gravedad(self) -> np.ndarray:
        """Aceleración virtual que, con la inversa de B, sostiene el peso."""
        v0 = np.zeros(len(self.k_gpi))
        if self.modelo is not None:
            m_cal, g_cal = self._modelo_cal(self.q, self.data_ctrl)
            m_ctrl = escalar_beta(m_cal, self.beta[self.g_mod])
            v0[self.g_mod] = np.linalg.solve(m_ctrl, g_cal)
        return v0

    def _sostener_aqui(self, motivo: str) -> None:
        """Aborta el movimiento y sostiene la postura medida actual.

        La referencia pasa a ser la posición medida (error cero) y los
        integradores se reinician para aportar la parte de la gravedad que
        no da el feedforward, así la corriente no salta ni cae a cero.
        """
        self.objetivo = self.q.copy()
        self.q_d = self.q.copy()
        self.qd_d[:] = 0.0
        self.qdd_d[:] = 0.0
        self.traj.fijar(self.q)
        self.shaper.reiniciar()
        self._mov = None
        self.gpi.reiniciar(self._v_gravedad() * (1.0 - self.r_grav))
        self._evento(f'Movimiento abortado: {motivo}. Se sostiene la postura '
                     f'actual (si se repite en 3 s, falla y corriente cero)')

    def deshabilitar(self, motivo: str = 'pedido') -> None:
        """Abre el lazo (corriente cero)."""
        if self.estado in ('habilitado', 'falla'):
            self.estado = 'deshabilitado'
            self.motivo = motivo
            self._evento(f'Control deshabilitado ({motivo})')

    def paro(self, motivo: str = 'paro de emergencia') -> None:
        """Paro de emergencia (latched)."""
        if self.estado != 'paro':
            self._evento(f'PARO DE EMERGENCIA: {motivo}')
        self.estado = 'paro'
        self.motivo = motivo
        self.traj.fijar(self.q_d)
        self.objetivo = self.q_d.copy()
        self._mov = None

    def liberar_paro(self) -> Tuple[bool, str]:
        """Sale del paro a 'deshabilitado' (habilitar es otro paso)."""
        if self.estado != 'paro':
            return False, 'No hay paro activo'
        self.estado = 'deshabilitado'
        self.motivo = 'paro liberado'
        self._evento('Paro liberado: el control queda deshabilitado; '
                     'habilítalo para mover el brazo')
        return True, 'Paro liberado'

    def _falla(self, motivo: str) -> None:
        if self.estado != 'falla':
            self._evento(f'FALLA: {motivo} -> corriente cero')
        self.estado = 'falla'
        self.motivo = motivo
        self.traj.fijar(self.q)
        self._mov = None

    # ------------------------------------------------------------------
    # Objetivos y trayectoria
    # ------------------------------------------------------------------
    def _estado_referencia(self, t: float):
        """Estado (q, q', q'', q''', q'''') de la referencia CRUDA en t."""
        return self.traj.estado_en(t)

    def planear(self, objetivo_usr: np.ndarray, pct: Optional[float] = None,
                t: Optional[float] = None
                ) -> Tuple[bool, str, Optional[TrayectoriaNonica]]:
        """Planea (sin aplicar) un movimiento a `objetivo_usr` [rad,
        coordenadas del usuario]; NaN = conservar el objetivo actual.

        Returns:
            (ok, mensaje, trayectoria) — aplicar con aplicar_trayectoria().
        """
        if self.estado != 'habilitado':
            return False, f'Control {self.estado}: objetivo ignorado', None
        t = self.t_med if t is None else t
        objetivo_usr = np.asarray(objetivo_usr, dtype=float)
        actual_usr = self.a_usuario(self.objetivo)
        objetivo_usr = np.where(np.isnan(objetivo_usr), actual_usr,
                                objetivo_usr)
        objetivo = self.desde_usuario(objetivo_usr)
        try:
            objetivo, avisos = validar_objetivo(
                objetivo, self.lo, self.hi, self.margen,
                self.cfg.trayectoria['objetivo_fuera_de_rango'], self.nombres)
        except ValueError as exc:
            return False, str(exc), None
        pct = self.pct_defecto if pct is None or np.isnan(pct) else float(pct)
        pct_ok = float(np.clip(pct, self.pct_min, self.pct_max))
        if pct_ok != pct:
            avisos.append(f'velocidad {pct:.0f} % fuera de rango; uso '
                          f'{pct_ok:.0f} %')
        q0, v0, a0, j0, s0 = self._estado_referencia(t)
        nueva = TrayectoriaNonica(self.traj.v_max, self.traj.a_max,
                                  self.traj.t_min, self.traj.j_max)
        t_modo = 0.0
        if self.shaper.activo:
            t_modo = duracion_minima_modo(
                self.shaper.frecuencia_hz,
                float(self.cfg.shaper['ciclos_modo_min']))
        escala = pct_ok / 100.0
        T = nueva.planear(t, q0, v0, a0, objetivo, escala, j0, s0, t_modo)
        alargada = False
        if self.modelo is not None:
            for _ in range(5):
                t_req, msg = self._duracion_por_corriente(nueva)
                if msg:
                    return False, msg, None
                if t_req <= T * 1.001:
                    break
                T = nueva.planear(t, q0, v0, a0, objetivo, escala, j0, s0,
                                  max(t_modo, t_req * 1.02))
                alargada = True
        texto = (f'Objetivo {np.round(np.degrees(self.a_usuario(objetivo)), 2).tolist()}° '
                 f'a {pct_ok:.0f} % (duración {T:.2f} s)')
        if alargada:
            texto += ' — alargada para no exceder la corriente'
        if avisos:
            texto += ' | ' + '; '.join(avisos)
        return True, texto, nueva

    def _duracion_por_corriente(self, tr: TrayectoriaNonica,
                                n_muestras: int = 40) -> Tuple[float, str]:
        """Duración mínima para que la corriente nominal (cota alta del
        modelo) quepa en margen * límite. Devuelve (T_min, error)."""
        lazo = self.cfg.lazo
        f_m = float(lazo['factor_inercia_sup'])
        f_g = float(lazo['factor_gravedad_sup'])
        sel = self.g_mod
        lim = (float(lazo['margen_corriente_trayectoria'])
               * self.limite[sel] * self.factor_limite[sel])
        k = self.k_tau[sel]
        fc = self.fc[sel]
        T0 = tr.duration
        t_req = 0.0
        for tau in np.linspace(0.0, T0, n_muestras):
            q, _, qdd = tr.evaluar(tau)
            m_cal, g_cal = self._modelo_cal(q, self.data_plan)
            qdd_m = qdd[self.k_gpi[sel]]
            a_term = np.abs(f_m * (m_cal @ qdd_m)) / k
            b_nom = np.abs(g_cal) / k + fc
            b_term = np.abs(f_g * g_cal) / k + fc
            lim_fisico = self.limite[sel] * self.factor_limite[sel]
            if np.any(b_nom >= lim_fisico):
                j = int(np.argmax(b_nom - lim_fisico))
                nom = self.movibles[self.k_gpi[sel[j]]].nombre
                return 0.0, (f'Objetivo rechazado: sostener el brazo en una '
                             f'postura de la trayectoria pide {b_nom[j]:.2f} A '
                             f'en {nom} (límite {lim_fisico[j]:.2f} A)')
            # corriente disponible para acelerar; si la cota alta de la
            # gravedad ya se come el margen, se deja una fracción del hueco
            # físico (movimiento muy lento pero factible)
            margen = np.maximum(lim - b_term, 0.15 * (lim_fisico - b_nom))
            ratio = np.max(a_term / margen)
            if ratio > 1.0:
                t_req = max(t_req, T0 * math.sqrt(ratio))
        return t_req, ''

    def aplicar_trayectoria(self, tr: TrayectoriaNonica) -> None:
        """Sustituye la trayectoria en curso (rápido; llamar con lock)."""
        cambio = np.abs(tr.q_final - self.objetivo) > 1e-9
        self.traj = tr
        self.objetivo = tr.q_final.copy()
        # un servo de posición directa sólo se empieza a mandar cuando se
        # le pide un objetivo (antes no se sabe dónde está si no se mide)
        self.pos_publicar[self.k_pos] |= cambio[self.k_pos]
        mov0 = self.q.copy()
        self._mov = {'q0': mov0, 'qf': tr.q_final.copy(),
                     't_fin': tr.t0 + tr.duration + self.shaper.retardo,
                     'muestras': [], 'grabando': False}

    def mover(self, objetivo_usr: np.ndarray, pct: Optional[float] = None
              ) -> Tuple[bool, str]:
        """planear() + aplicar_trayectoria() en un solo paso."""
        ok, msg, tr = self.planear(objetivo_usr, pct)
        if ok:
            self.aplicar_trayectoria(tr)
        self.log(msg) if ok else self.aviso(msg)
        return ok, msg

    def objetivo_por_nombres(self, objetivos: Dict[str, float]) -> np.ndarray:
        """Vector de objetivos (usuario, rad) con NaN donde no se indica."""
        v = np.full(self.n, np.nan)
        for nom, val in objetivos.items():
            if nom not in self.nombres:
                raise KeyError(f'"{nom}" no es una articulación activa '
                               f'({self.nombres})')
            v[self.nombres.index(nom)] = val
        return v

    # ------------------------------------------------------------------
    # Lazo
    # ------------------------------------------------------------------
    def paso(self, t: Optional[float] = None) -> Mando:
        """Un paso del lazo con la última medición (tiempo t)."""
        t = self.t_med if t is None else t
        dt = self.dt_nominal if self.t_prev is None else t - self.t_prev
        dt = float(np.clip(dt, 0.2 * self.dt_nominal, 3.0 * self.dt_nominal))
        self.t_prev = t
        self.watchdog = 'ok'
        qd_est = self.vel.actualizar(self.q, dt)

        # referencia
        if self.traj.active:
            q_c, qd_c, qdd_c, _, _ = self.traj.estado_en(t)
        else:
            q_c, qd_c, qdd_c = (self.traj.q_final.copy(), np.zeros(self.n),
                                np.zeros(self.n))
        self.q_d, self.qd_d, self.qdd_d = self.shaper.aplicar(t, q_c, qd_c,
                                                              qdd_c)
        if self.estado != 'habilitado':
            return self._mando_abierto(t)
        cmd = self._ley(self.q, qd_est, dt, congelar=False)
        self._post_paso(t, cmd)
        return self._mando(cmd)

    def paso_sin_medicion(self, edad: float) -> Optional[Mando]:
        """Llamar periódicamente si no llega medición (edad en s).

        Returns:
            Mando a publicar, o None si no hay que hacer nada.
        """
        seg = self.cfg.seguridad
        if edad > float(seg['watchdog_quitar']):
            self.watchdog = 'quitar'
            if self.estado == 'habilitado':
                self._falla(f'sin medición por {edad * 1000:.0f} ms')
            return self._mando_abierto(self.t_prev or 0.0)
        if edad > float(seg['watchdog_congelar']):
            self.watchdog = 'congelar'
            if self.estado != 'habilitado':
                return None
            cmd = self._ley(self.q, np.zeros(self.n), 0.0, congelar=True)
            return self._mando(cmd)
        return None

    def _ley(self, q: np.ndarray, qd_est: np.ndarray, dt: float,
             congelar: bool) -> np.ndarray:
        """Ley de control: devuelve el mando de las articulaciones GPI."""
        lazo = self.cfg.lazo
        kg = self.k_gpi
        e = q[kg] - self.q_d[kg]
        t_ahora = self.t_prev if self.t_prev is not None else 0.0
        en_gracia = (self._t_aborto is not None
                     and t_ahora - self._t_aborto < self.gracia_aborto)
        # justo después de abortar, el brazo trae inercia: se tolera el
        # doble de error mientras frena
        error_max = self.error_max * (2.0 if en_gracia else 1.0)
        if np.any(np.abs(e) > error_max) and not congelar:
            j = int(np.argmax(np.abs(e)))
            motivo = (f'error de seguimiento de {math.degrees(e[j]):.1f}° en '
                      f'{self.nombres[kg[j]]}')
            t = self.t_prev if self.t_prev is not None else 0.0
            reciente = (self._t_aborto is not None
                        and t - self._t_aborto < 3.0)
            if self.accion_error == 'sostener' and not reciente:
                self._t_aborto = t
                self._sostener_aqui(motivo)
                e = q[kg] - self.q_d[kg]          # = 0
            else:
                self._falla(motivo)
                return np.zeros(len(kg))
        ff = 1.0 if lazo['feedforward_aceleracion'] else 0.0
        if congelar:
            v = self.gpi.parte_integral()
            s2_dot = np.zeros(len(kg))
        else:
            v, s2_dot = self.gpi.calcular(e, ff * self.qdd_d[kg])
            v = v + aceleracion_pared(q[kg], qd_est[kg], self.lo[kg],
                                      self.hi[kg], self.margen[kg],
                                      self.k_pared[kg], self.d_pared[kg])

        tau = np.zeros(len(kg))
        m_diag = self.beta * self.j_const          # para anti-windup
        sel = self.g_mod
        if self.modelo is not None and sel.size:
            m_cal, g_cal = self._modelo_cal(q, self.data_ctrl)
            m_ctrl = escalar_beta(m_cal, self.beta[sel])
            # rampa de gravedad con traspaso desde los integradores
            if not congelar and self.r_grav < 1.0:
                rampa = float(lazo['rampa_gravedad'])
                dr = min(1.0 - self.r_grav, dt / max(rampa, 1e-6))
                if dr > 0.0:
                    traspaso = np.zeros(len(kg))
                    traspaso[sel] = -np.linalg.solve(m_ctrl, g_cal) * dr
                    self.gpi.ajustar_s1(traspaso)
                    self.r_grav += dr
                    if not congelar:      # v cambió con el traspaso
                        v[sel] = v[sel] + traspaso[sel]
            tau[sel] = m_ctrl @ v[sel] + self.r_grav * g_cal
            m_diag = m_diag.copy()
            m_diag[sel] = np.diag(m_ctrl)
        sin = self.g_sin
        tau[sin] = self.beta[sin] * self.j_const[sin] * v[sin]
        # fricción viscosa conocida (p. ej. <dynamics damping> del URDF)
        tau = tau + self.b_visc * self.qd_d[kg]

        k = np.where(self.es_corriente, self.k_tau, 1.0)
        cmd_core = tau / k
        if self.friccion_velocidad:
            comp = self.fc * np.tanh(self.qd_d[kg] / 0.02)
        else:
            comp = self.fc * np.tanh(self.fc_gain * cmd_core)
        cmd_libre = cmd_core + comp
        lim = self.limite * self.factor_limite
        cmd = np.clip(cmd_libre, -lim, lim)
        cmd = self.pendiente.aplicar(cmd, dt if dt > 0 else self.dt_nominal)
        self.saturado = np.abs(cmd - cmd_libre) > 1e-9
        self.saturaciones += self.saturado.astype(int)
        if not congelar and dt > 0.0:
            dv = (cmd - cmd_libre) * k / np.maximum(m_diag, 1e-9)
            self.gpi.integrar(s2_dot, dt, None, np.where(self.saturado, dv,
                                                         0.0))
        self._ultimo = {'e': e, 'v': v, 'cmd_core': cmd_core, 'comp': comp,
                        'cmd': cmd, 'tau': tau}
        return cmd

    def _mando(self, cmd_gpi: np.ndarray) -> Mando:
        """Arma el mando completo (GPI + posición directa) con signos."""
        valores = np.zeros(self.n)
        valores[self.k_gpi] = self.signo[self.k_gpi] * cmd_gpi
        kp = self.k_pos
        if kp.size:
            raw = self.signo[kp] * (self.q_d[kp] - self.offset[kp])
            valores[kp] = np.where(self.unidad_deg[kp], np.degrees(raw), raw)
        publicar = np.ones(self.n, dtype=bool)
        publicar[kp] = self.pos_publicar[kp]
        self.ultimo_mando = valores
        return Mando(valores, publicar,
                     [a.mando for a in self.movibles])

    def _mando_abierto(self, t: float) -> Mando:
        """Mando sin lazo: cero en corriente; posición directa se queda."""
        if self.estado == 'paro' and self.paro_accion == 'congelar' and (
                self.watchdog == 'ok'):
            cmd = self._ley(self.q, self.vel.v, self.dt_nominal, False)
            return self._mando(cmd)
        valores = np.zeros(self.n)
        publicar = np.ones(self.n, dtype=bool)
        publicar[self.k_pos] = False         # no mover servos sin lazo
        self.ultimo_mando = valores
        self.saturado[:] = False
        return Mando(valores, publicar, [a.mando for a in self.movibles])

    # ------------------------------------------------------------------
    # Después de cada paso: supervisor, kappa, registro
    # ------------------------------------------------------------------
    def _post_paso(self, t: float, cmd: np.ndarray) -> None:
        phi = self.gpi.phi_hat()
        kg = self.k_gpi
        self.registro_phi.append((t, phi.copy()))
        mov = self._mov
        if mov is not None and not mov['grabando']:
            u = getattr(self, '_ultimo', None)
            if u is not None:
                mov.setdefault('acel', []).append(
                    (self.qdd_d[kg].copy(), u['v'].copy()))
        if mov is not None:
            if not mov['grabando'] and t >= mov['t_fin']:
                mov['grabando'] = True
                mov['t_inicio_ventana'] = t
                self.t_fin_movimiento = t
            if mov['grabando']:
                mov['muestras'].append(self.q.copy())
                if t - mov['t_inicio_ventana'] >= self.supervisor.ventana:
                    self._evaluar_movimiento(mov)
                    self._mov = None
        if self.observador is not None:
            self.observador(self.datos_paso(t))

    def _evaluar_movimiento(self, mov: dict) -> None:
        qs = np.array(mov['muestras'])
        acel = mov.get('acel') or []
        if acel:
            qdd = np.array([a[0] for a in acel])
            vv = np.array([a[1] for a in acel])
            for g in range(len(self.k_gpi)):
                self.kappa[g] = estimar_kappa_lote(qdd[:, g], vv[:, g])
        kappas = self.kappa
        for g, k in enumerate(self.k_gpi):
            if abs(mov['qf'][k] - mov['q0'][k]) < self.supervisor.desplazamiento_min:
                continue          # casi no se movió: el % no tiene sentido
            r = analizar(self.nombres[k], qs[:, k], mov['q0'][k],
                         mov['qf'][k], self.supervisor.banda)
            self.sobreimpulso[k] = math.radians(r.sobreimpulso_deg)
            kap = kappas[g] if np.isfinite(kappas[g]) else None
            n_prev = len(self.supervisor.eventos)
            factor = self.supervisor.decidir(r, float(self.gpi.wn()[g]), kap,
                                             float(self.wn_inicial[g]))
            for ev in self.supervisor.eventos[n_prev:]:
                self._evento('Supervisor: ' + ev)
            if factor is not None:
                self.gpi.escalar(g, factor)

    def datos_paso(self, t: float) -> dict:
        """Datos del último paso (para autotune y registro)."""
        u = getattr(self, '_ultimo', None) or {}
        return {'t': t, 'q': self.q.copy(), 'q_d': self.q_d.copy(),
                'qd_d': self.qd_d.copy(), 'qdd_d': self.qdd_d.copy(),
                'cmd_core': u.get('cmd_core'), 'cmd': u.get('cmd'),
                'e': u.get('e'), 'saturado': self.saturado.copy(),
                'estado': self.estado}

    # ------------------------------------------------------------------
    # Ajustes en vivo
    # ------------------------------------------------------------------
    def fijar_polos(self, nombre: str, polos) -> None:
        """Cambia los polos de una articulación GPI."""
        self.gpi.fijar_polos(self.nombres_gpi_index(nombre), polos)

    def fijar(self, nombre: str, campo: str, valor: float) -> None:
        """Cambia beta, alfa, armadura, alfa_gravedad, friccion_coulomb o
        limite_mando de una articulación GPI (ya validado)."""
        g = self.nombres_gpi_index(nombre)
        destino = {'beta': self.beta, 'alfa': self.alfa,
                   'armadura': self.armadura, 'alfa_gravedad': self.alfa_g,
                   'friccion_coulomb': self.fc,
                   'limite_mando': self.limite}[campo]
        destino[g] = float(valor)

    def configurar_shaper(self, tipo: str, frecuencia_hz: float,
                          zeta: float, tolerancia: float) -> None:
        """Cambia el conformador (sólo con el brazo quieto)."""
        if self.traj.active:
            raise RuntimeError('No se puede cambiar el conformador con el '
                               'brazo en movimiento')
        self.shaper.configurar(tipo, frecuencia_hz, zeta, tolerancia)
        self.cfg.shaper.update({'tipo': tipo, 'frecuencia_hz': frecuencia_hz,
                                'amortiguamiento': zeta,
                                'tolerancia_ei': tolerancia})

    def movimiento_terminado(self) -> bool:
        """True si no hay trayectoria ni conformado pendiente."""
        if self.traj.active:
            return False
        return self._mov is None or self._mov.get('grabando', False)

    # ------------------------------------------------------------------
    def diagnostico(self) -> Diagnostico:
        """Instantánea del estado para publicar."""
        d = self.diag
        d.estado = self.estado
        d.watchdog = self.watchdog
        d.q = self.a_usuario(self.q)
        d.q_ref = self.a_usuario(self.q_d)
        d.error = self.q - self.q_d
        d.mando = self.ultimo_mando.copy()
        n = self.n
        d.phi_hat = np.full(n, np.nan)
        d.phi_hat[self.k_gpi] = self.gpi.phi_hat()
        d.kappa = np.full(n, np.nan)
        d.kappa[self.k_gpi] = self.kappa
        d.saturaciones = np.zeros(n, dtype=int)
        d.saturaciones[self.k_gpi] = self.saturaciones
        d.saturado = np.zeros(n, dtype=bool)
        d.saturado[self.k_gpi] = self.saturado
        d.sobreimpulso = self.sobreimpulso.copy()
        d.wn = np.full(n, np.nan)
        d.wn[self.k_gpi] = self.gpi.wn()
        d.beta = np.full(n, np.nan)
        d.beta[self.k_gpi] = self.beta
        d.trayectoria_activa = bool(self.traj.active)
        d.rampa_gravedad = self.r_grav
        return d
