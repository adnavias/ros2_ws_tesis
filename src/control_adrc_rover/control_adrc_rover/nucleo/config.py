"""Carga, fusión y validación de la configuración del brazo.

La configuración se arma en capas (cada una sobrescribe a la anterior):

    1. Perfil del brazo      config/brazos/<brazo>.yaml
    2. Plataforma            config/plataformas/<sim|real>.yaml
    3. Ajustes guardados     config/autotune_<brazo>.yaml   (si existe)
    4. Sobrescrituras        parámetros `montadas` y `estados` del nodo

Dentro de cada capa, la sección `por_defecto` se aplica a todas las
articulaciones y `articulaciones.<nombre>` a una sola. El número de
articulaciones n sale del perfil (orden = orden de `articulaciones`).

Estados de una articulación:
    activa     se mueve y (si su mando lo usa) entra al modelo dinámico.
    bloqueada  existe y está fija en `posicion_bloqueo` [rad, URDF]; su
               masa cuenta en el modelo; no se manda nada.
    ausente    no está montada: su subárbol no tiene masa en el modelo y
               no se manda nada. Todo lo que cuelga de ella debe estar
               también ausente.

Tipos de mando (sólo para articulaciones activas):
    corriente_con_modelo  GPI + inversa de B con M(q), g(q); sale [A].
    torque_sin_modelo     GPI con inercia constante; sale [N·m].
    posicion_directa      se publica la referencia de posición (servo).

Encoders (articulaciones con `medida: true`):
    incremental  leen 0 al encender: q_urdf = signo*q_raw + postura_encendido_urdf
    absoluto     ya vienen en convención URDF: q_urdf = signo*q_raw + offset_absoluto

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import copy
import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml

ESTADOS = ('activa', 'bloqueada', 'ausente')
MANDOS = ('corriente_con_modelo', 'torque_sin_modelo', 'posicion_directa')
ENCODERS = ('incremental', 'absoluto')
TIPOS_MANDO_ROS = ('float64_por_articulacion', 'float64_multiarray',
                   'joint_state')
SHAPERS = ('ninguno', 'zv', 'zvd', 'ei')
PAROS = ('sin_corriente', 'congelar')
REFERENCIAS = ('relativa_arranque', 'urdf')


class ErrorConfig(ValueError):
    """Error de configuración con mensaje pensado para el usuario."""


# ----------------------------------------------------------------------
# Utilidades de YAML
# ----------------------------------------------------------------------
def fusionar(base: dict, encima: Optional[dict]) -> dict:
    """Fusión profunda: los dicts se combinan, el resto se reemplaza."""
    out = copy.deepcopy(base)
    for clave, valor in (encima or {}).items():
        if isinstance(valor, dict) and isinstance(out.get(clave), dict):
            out[clave] = fusionar(out[clave], valor)
        else:
            out[clave] = copy.deepcopy(valor)
    return out


def leer_yaml(ruta: str) -> dict:
    """Lee un YAML y devuelve un dict (vacío si el archivo está vacío)."""
    if not os.path.isfile(ruta):
        raise ErrorConfig(f'No existe el archivo de configuración: {ruta}')
    with open(ruta, 'r', encoding='utf-8') as f:
        try:
            datos = yaml.safe_load(f) or {}
        except yaml.YAMLError as exc:
            raise ErrorConfig(f'{ruta}: YAML inválido: {exc}') from exc
    if not isinstance(datos, dict):
        raise ErrorConfig(f'{ruta}: se esperaba un diccionario en la raíz')
    return datos


def resolver_ruta(ruta: str, buscar_paquete=None) -> str:
    """Resuelve package://paquete/sub/ruta y ~ en rutas de archivos."""
    if ruta.startswith('package://'):
        resto = ruta[len('package://'):]
        paquete, _, sub = resto.partition('/')
        if buscar_paquete is None:
            from ament_index_python.packages import (
                get_package_share_directory)
            buscar_paquete = get_package_share_directory
        return os.path.join(buscar_paquete(paquete), sub)
    return os.path.expanduser(ruta)


# ----------------------------------------------------------------------
# Estructuras
# ----------------------------------------------------------------------
@dataclass
class Articulacion:
    """Configuración final (ya fusionada y validada) de una articulación."""

    nombre: str
    indice: int
    estado: str
    mando: str
    medida: bool
    # Motor / actuador
    motor: str
    k_tau: float
    armadura_nominal: float
    limite_mando: float
    corriente_vacio: float
    ancho_friccion: float
    pendiente_max: float           # [A/s] (o N·m/s, rad/s) 0 = sin límite
    inercia_constante: float
    # Interfaz y medición
    signo: float
    encoder: str
    postura_encendido_urdf: float
    offset_absoluto: float
    nombre_en_estado: str
    nombre_en_mando: str
    unidad_posicion: str
    # Modelo
    posicion_bloqueo: float
    # Trayectoria
    v_max_fisica: float
    a_max_100pct: float
    jerk_max_100pct: float
    # Control (GPI)
    polos: List[float]
    beta: float
    # Calibración (autotune; editables a mano)
    alfa: float
    armadura: float                # la que usa el modelo del control
    friccion_coulomb: float        # [A] (o N·m) compensación de fricción
    alfa_gravedad: float
    # Seguridad
    margen_limite: float           # [rad] pared virtual antes del límite URDF
    rigidez_pared: float           # [1/s^2] aceleración por rad de penetración
    amortiguamiento_pared: float   # [1/s]
    salto_max: float               # [rad] salto máximo por muestra
    desenvolver: Optional[bool]    # None = automático (continuous)
    # Límites del URDF (los completa el modelo)
    limite_inferior: float = -math.inf
    limite_superior: float = math.inf
    continua: bool = False

    @property
    def movible(self) -> bool:
        """True si se le generan trayectorias (activa)."""
        return self.estado == 'activa'

    @property
    def en_modelo(self) -> bool:
        """True si es coordenada libre del modelo dinámico reducido."""
        return self.estado == 'activa' and self.mando == 'corriente_con_modelo'

    @property
    def usa_gpi(self) -> bool:
        """True si se controla con el lazo GPI (no posición directa)."""
        return self.estado == 'activa' and self.mando != 'posicion_directa'

    @property
    def requiere_medicion(self) -> bool:
        """True si el control debe recibir su posición para arrancar."""
        return self.estado == 'activa' and self.medida


@dataclass
class ConfigBrazo:
    """Configuración completa del brazo para el control."""

    nombre: str
    urdf: str
    articulaciones: List[Articulacion]
    interfaz: Dict[str, Any]
    lazo: Dict[str, Any]
    trayectoria: Dict[str, Any]
    seguridad: Dict[str, Any]
    shaper: Dict[str, Any]
    supervisor: Dict[str, Any]
    autotune: Dict[str, Any]
    diagnostico: Dict[str, Any]
    archivos: List[str] = field(default_factory=list)
    crudo: Dict[str, Any] = field(default_factory=dict)

    # --- vistas útiles ---
    @property
    def nombres(self) -> List[str]:
        """Nombres de todas las articulaciones del perfil."""
        return [a.nombre for a in self.articulaciones]

    def por_estado(self, estado: str) -> List[Articulacion]:
        """Articulaciones en un estado dado."""
        return [a for a in self.articulaciones if a.estado == estado]

    @property
    def movibles(self) -> List[Articulacion]:
        """Articulaciones activas, en orden."""
        return [a for a in self.articulaciones if a.movible]

    def articulacion(self, nombre: str) -> Articulacion:
        """Busca una articulación por nombre."""
        for a in self.articulaciones:
            if a.nombre == nombre:
                return a
        raise ErrorConfig(f'No existe la articulación "{nombre}" en el perfil')

    def resumen(self) -> str:
        """Texto corto con el estado de cada articulación."""
        partes = []
        for a in self.articulaciones:
            extra = f'/{a.mando}' if a.estado == 'activa' else ''
            partes.append(f'{a.nombre}={a.estado}{extra}')
        return ', '.join(partes)


# ----------------------------------------------------------------------
# Valores por defecto de cada articulación
# ----------------------------------------------------------------------
DEFAULTS_ARTICULACION: Dict[str, Any] = {
    'estado': 'activa',
    'mando': 'corriente_con_modelo',
    'medida': True,
    'motor': '',
    'limite_mando': 1.0,
    'ancho_friccion': 0.1,
    'pendiente_max': 0.0,
    'inercia_constante': 1.0,
    'signo': 1.0,
    'encoder': 'incremental',
    'postura_encendido_urdf': 0.0,
    'offset_absoluto': 0.0,
    'nombre_en_estado': '',
    'nombre_en_mando': '',
    'unidad_posicion': 'rad',
    'posicion_bloqueo': 0.0,
    'v_max_fisica': 1.0,
    'a_max_100pct': 2.0,
    'jerk_max_100pct': 0.0,
    'polos': [],
    'wn': 10.0,
    'relacion_observador': 5.0,
    'beta': 1.0,
    'alfa': 1.0,
    'armadura': None,
    'friccion_coulomb': None,
    'alfa_gravedad': 1.0,
    'margen_limite': 0.05,
    'rigidez_pared': 400.0,
    'amortiguamiento_pared': 40.0,
    'salto_max': 0.5,
    'desenvolver': None,
}

DEFAULTS_SECCIONES: Dict[str, Dict[str, Any]] = {
    'interfaz': {
        'topico_estado': '/joint_states',
        'qos_estado_best_effort': False,
        'tipo_mando': 'float64_por_articulacion',
        'patron_topico_mando': '/brazo/{articulacion}/cmd_actuador',
        'topico_mando_unico': '/brazo/cmd_actuador',
        'patron_topico_fuerza': '/model/mi_rover/joint/{articulacion}/cmd_force',
        'topico_objetivo_deg': '/brazo/objetivo_deg',
        'topico_objetivo_rad': '/target_joint_states',
        'topico_objetivo_nombres_deg': '/brazo/objetivo_nombres_deg',
        'topico_heartbeat': '/brazo/interfaz/heartbeat',
        'usar_stamp_medicion': True,
    },
    'lazo': {
        'frecuencia_control': 200.0,
        'referencia_objetivos': 'relativa_arranque',
        'feedforward_gravedad': True,
        'feedforward_aceleracion': True,
        'rampa_gravedad': 1.0,
        'antiwindup_ganancia': 1.0,
        'factor_inercia_sup': 1.25,
        'factor_gravedad_sup': 1.15,
        'margen_corriente_trayectoria': 0.9,
        'incluir_armadura': True,
        'habilitar_al_arrancar': True,
    },
    'trayectoria': {
        'factor_100pct': 0.5,
        'velocidad_pct_defecto': 50.0,
        'velocidad_pct_min': 1.0,
        'duracion_minima': 0.5,
        'objetivo_fuera_de_rango': 'recortar',
    },
    'seguridad': {
        'watchdog_congelar': 0.02,
        'watchdog_quitar': 0.10,
        'paro_accion': 'sin_corriente',
        'heartbeat_timeout': 0.5,
        'muestras_invalidas_max': 10,
        'error_max_deg': 30.0,
    },
    'shaper': {
        'tipo': 'ninguno',
        'frecuencia_hz': 2.0,
        'amortiguamiento': 0.05,
        'tolerancia_ei': 0.05,
        'ciclos_modo_min': 1.0,
    },
    'supervisor': {
        'habilitado': True,
        'umbral_sobreimpulso_deg': 0.5,
        'umbral_sobreimpulso_pct': 2.0,
        'cruces_max': 2,
        'factor_reduccion': 0.85,
        'wn_min': 2.0,
        'ventana': 1.5,
        'banda_cruces_deg': 0.05,
    },
    'autotune': {
        'factor_corriente': 0.6,
        'amplitud_deg': 4.0,
        'velocidad_pct': 60.0,
        'repeticiones': 3,
        'error_aborto_deg': 8.0,
        'k_objetivo': 1.0,
        'paso_wn': 1.25,
        'wn_max': 30.0,
        'margen_wn': 0.35,
        'amplitud_wn_deg': 2.0,
        'posturas': {},
    },
    'diagnostico': {
        'publicar': True,
        'frecuencia': 50.0,
        'periodo_log': 0.0,
    },
}


# ----------------------------------------------------------------------
# Construcción
# ----------------------------------------------------------------------
def _num(valor, nombre: str, ctx: str) -> float:
    try:
        x = float(valor)
    except (TypeError, ValueError) as exc:
        raise ErrorConfig(f'{ctx}: "{nombre}" debe ser un número '
                          f'(llegó {valor!r})') from exc
    if math.isnan(x):
        raise ErrorConfig(f'{ctx}: "{nombre}" no puede ser NaN')
    return x


def _elegir(valor, opciones, nombre: str, ctx: str) -> str:
    if valor not in opciones:
        raise ErrorConfig(f'{ctx}: {nombre}="{valor}" no válido; usa uno de '
                          f'{list(opciones)}')
    return valor


def polos_desde(datos: dict) -> List[float]:
    """Polos [λ1..λ4] (rad/s) desde `polos` o desde wn y la relación."""
    polos = datos.get('polos') or []
    if polos:
        return [float(p) for p in polos]
    wn = float(datos.get('wn', 10.0))
    r = float(datos.get('relacion_observador', 5.0))
    return [wn, wn, r * wn, r * wn]


def construir(capas: List[dict], nombres_archivo: List[str],
              montadas: int = 0, estados: Optional[Dict[str, str]] = None,
              buscar_paquete=None) -> ConfigBrazo:
    """Fusiona las capas y devuelve la configuración validada."""
    datos: dict = {}
    for capa in capas:
        datos = fusionar(datos, capa)
    nombre = str(datos.get('nombre', 'brazo'))
    if 'urdf' not in datos:
        raise ErrorConfig('El perfil debe indicar "urdf" (ruta o package://)')
    urdf = resolver_ruta(str(datos['urdf']), buscar_paquete)

    arts_datos = datos.get('articulaciones') or {}
    if not isinstance(arts_datos, dict) or not arts_datos:
        raise ErrorConfig('El perfil debe tener la sección "articulaciones" '
                          'con al menos una articulación')
    motores = datos.get('motores') or {}
    por_defecto = fusionar(DEFAULTS_ARTICULACION,
                           datos.get('por_defecto') or {})

    # Sobrescrituras de estado: montadas=N deja ausentes desde la N+1.
    nombres = list(arts_datos.keys())
    estados = dict(estados or {})
    if montadas and montadas > 0:
        if montadas > len(nombres):
            raise ErrorConfig(f'montadas={montadas} pero el perfil tiene '
                              f'{len(nombres)} articulaciones')
        for nom in nombres[montadas:]:
            estados.setdefault(nom, 'ausente')
    for nom in estados:
        if nom not in nombres:
            raise ErrorConfig(f'estados: la articulación "{nom}" no está en '
                              f'el perfil ({nombres})')

    articulaciones: List[Articulacion] = []
    for idx, nom in enumerate(nombres):
        crudo = fusionar(por_defecto, arts_datos[nom] or {})
        if nom in estados:
            crudo['estado'] = estados[nom]
        articulaciones.append(
            _construir_articulacion(nom, idx, crudo, motores))

    secciones = {}
    for sec, defaults in DEFAULTS_SECCIONES.items():
        secciones[sec] = fusionar(defaults, datos.get(sec) or {})

    cfg = ConfigBrazo(
        nombre=nombre, urdf=urdf, articulaciones=articulaciones,
        archivos=list(nombres_archivo), crudo=datos, **secciones)
    validar(cfg)
    return cfg


def _construir_articulacion(nom: str, idx: int, d: dict,
                            motores: dict) -> Articulacion:
    ctx = f'articulaciones.{nom}'
    estado = _elegir(d['estado'], ESTADOS, 'estado', ctx)
    mando = _elegir(d['mando'], MANDOS, 'mando', ctx)
    encoder = _elegir(d['encoder'], ENCODERS, 'encoder', ctx)

    # Parámetros del motor (k_tau, armadura nominal, i0) si se indicó uno.
    k_tau, armadura_nom, i0 = 1.0, 0.0, 0.0
    motor = str(d.get('motor') or '')
    if mando == 'corriente_con_modelo':
        if not motor:
            raise ErrorConfig(f'{ctx}: el mando "corriente_con_modelo" '
                              f'necesita "motor"')
        if motor not in motores:
            raise ErrorConfig(f'{ctx}: motor "{motor}" no está en la sección '
                              f'"motores" ({list(motores)})')
        m = motores[motor]
        ctxm = f'motores.{motor}'
        kt_caja = _num(m.get('k_t_caja'), 'k_t_caja', ctxm)
        n_s = _num(m.get('relacion_sinfin', 1.0), 'relacion_sinfin', ctxm)
        eta_s = _num(m.get('eficiencia_sinfin', 1.0), 'eficiencia_sinfin',
                     ctxm)
        n_c = _num(m.get('relacion_caja', 1.0), 'relacion_caja', ctxm)
        j_rot = _num(m.get('inercia_rotor', 0.0), 'inercia_rotor', ctxm)
        k_tau = kt_caja * n_s * eta_s
        armadura_nom = j_rot * (n_c * n_s) ** 2
        i0 = _num(m.get('corriente_vacio', 0.0), 'corriente_vacio', ctxm)
        i_cont = m.get('corriente_continua')
        par_cont = m.get('par_continuo_salida')
        lim = _num(d['limite_mando'], 'limite_mando', ctx)
        if i_cont is not None and lim > float(i_cont) + 1e-9:
            raise ErrorConfig(
                f'{ctx}: limite_mando={lim} A supera la corriente continua '
                f'del motor ({float(i_cont)} A)')
        if par_cont is not None and lim * k_tau > float(par_cont) + 1e-6:
            raise ErrorConfig(
                f'{ctx}: limite_mando={lim} A da {lim * k_tau:.1f} N·m '
                f'(K_tau={k_tau:.3f}) y supera el par continuo de la salida '
                f'({float(par_cont)} N·m). Máximo: '
                f'{float(par_cont) / k_tau:.2f} A')
    if d.get('k_tau') is not None:          # sobrescritura explícita
        k_tau = _num(d['k_tau'], 'k_tau', ctx)
    if d.get('corriente_vacio') is not None:
        i0 = _num(d['corriente_vacio'], 'corriente_vacio', ctx)

    armadura = d.get('armadura')
    armadura = armadura_nom if armadura is None else _num(
        armadura, 'armadura', ctx)
    fc = d.get('friccion_coulomb')
    fc = i0 if fc is None else _num(fc, 'friccion_coulomb', ctx)

    polos = polos_desde(d)
    if len(polos) != 4:
        raise ErrorConfig(f'{ctx}: "polos" debe tener 4 valores (llegaron '
                          f'{len(polos)})')
    if any(p <= 0.0 for p in polos):
        raise ErrorConfig(f'{ctx}: todos los polos deben ser positivos '
                          f'(son -λ en el plano s); llegaron {polos}')

    signo = _num(d['signo'], 'signo', ctx)
    if signo not in (1.0, -1.0):
        raise ErrorConfig(f'{ctx}: signo debe ser 1.0 o -1.0')
    unidad = _elegir(d['unidad_posicion'], ('rad', 'deg'), 'unidad_posicion',
                     ctx)
    desenv = d.get('desenvolver')

    art = Articulacion(
        nombre=nom, indice=idx, estado=estado, mando=mando,
        medida=bool(d['medida']), motor=motor, k_tau=k_tau,
        armadura_nominal=armadura_nom,
        limite_mando=_num(d['limite_mando'], 'limite_mando', ctx),
        corriente_vacio=i0,
        ancho_friccion=_num(d['ancho_friccion'], 'ancho_friccion', ctx),
        pendiente_max=_num(d['pendiente_max'], 'pendiente_max', ctx),
        inercia_constante=_num(d['inercia_constante'], 'inercia_constante',
                               ctx),
        signo=signo, encoder=encoder,
        postura_encendido_urdf=_num(d['postura_encendido_urdf'],
                                    'postura_encendido_urdf', ctx),
        offset_absoluto=_num(d['offset_absoluto'], 'offset_absoluto', ctx),
        nombre_en_estado=str(d['nombre_en_estado'] or nom),
        nombre_en_mando=str(d['nombre_en_mando'] or nom),
        unidad_posicion=unidad,
        posicion_bloqueo=_num(d['posicion_bloqueo'], 'posicion_bloqueo', ctx),
        v_max_fisica=_num(d['v_max_fisica'], 'v_max_fisica', ctx),
        a_max_100pct=_num(d['a_max_100pct'], 'a_max_100pct', ctx),
        jerk_max_100pct=_num(d['jerk_max_100pct'], 'jerk_max_100pct', ctx),
        polos=polos, beta=_num(d['beta'], 'beta', ctx),
        alfa=_num(d['alfa'], 'alfa', ctx), armadura=armadura,
        friccion_coulomb=fc,
        alfa_gravedad=_num(d['alfa_gravedad'], 'alfa_gravedad', ctx),
        margen_limite=_num(d['margen_limite'], 'margen_limite', ctx),
        rigidez_pared=_num(d['rigidez_pared'], 'rigidez_pared', ctx),
        amortiguamiento_pared=_num(d['amortiguamiento_pared'],
                                   'amortiguamiento_pared', ctx),
        salto_max=_num(d['salto_max'], 'salto_max', ctx),
        desenvolver=None if desenv is None else bool(desenv),
    )
    return art


def validar(cfg: ConfigBrazo) -> None:
    """Revisa rangos y coherencia; lanza ErrorConfig con mensaje claro."""
    for a in cfg.articulaciones:
        ctx = f'articulaciones.{a.nombre}'
        if a.limite_mando <= 0.0:
            raise ErrorConfig(f'{ctx}: limite_mando debe ser > 0')
        if not 0.05 <= a.beta <= 1.5:
            raise ErrorConfig(f'{ctx}: beta={a.beta} fuera de [0.05, 1.5]')
        if not 0.05 <= a.alfa <= 20.0:
            raise ErrorConfig(f'{ctx}: alfa={a.alfa} fuera de [0.05, 20]')
        if a.armadura < 0.0:
            raise ErrorConfig(f'{ctx}: armadura no puede ser negativa')
        if a.movible and (a.v_max_fisica <= 0.0 or a.a_max_100pct <= 0.0):
            raise ErrorConfig(f'{ctx}: v_max_fisica y a_max_100pct deben '
                              f'ser > 0')
        if a.mando == 'torque_sin_modelo' and a.inercia_constante <= 0.0:
            raise ErrorConfig(f'{ctx}: inercia_constante debe ser > 0')
    f = float(cfg.lazo['frecuencia_control'])
    if f <= 0.0:
        raise ErrorConfig('lazo.frecuencia_control debe ser > 0')
    dt = 1.0 / f
    for a in cfg.articulaciones:
        if a.usa_gpi and max(a.polos) * dt > 0.5:
            raise ErrorConfig(
                f'articulaciones.{a.nombre}: el polo más rápido '
                f'({max(a.polos):.1f} rad/s) da λ·dt = '
                f'{max(a.polos) * dt:.2f} > 0.5 a {f:.0f} Hz. Baja los polos '
                f'o sube la frecuencia.')
    _elegir(cfg.interfaz['tipo_mando'], TIPOS_MANDO_ROS, 'tipo_mando',
            'interfaz')
    _elegir(cfg.shaper['tipo'], SHAPERS, 'tipo', 'shaper')
    _elegir(cfg.seguridad['paro_accion'], PAROS, 'paro_accion', 'seguridad')
    _elegir(cfg.lazo['referencia_objetivos'], REFERENCIAS,
            'referencia_objetivos', 'lazo')
    _elegir(cfg.trayectoria['objetivo_fuera_de_rango'],
            ('recortar', 'rechazar'), 'objetivo_fuera_de_rango',
            'trayectoria')
    wc, wq = (float(cfg.seguridad['watchdog_congelar']),
              float(cfg.seguridad['watchdog_quitar']))
    if not 0.0 < wc < wq:
        raise ErrorConfig('seguridad: debe cumplirse 0 < watchdog_congelar '
                          '< watchdog_quitar')
    if not cfg.movibles:
        raise ErrorConfig('No hay ninguna articulación activa: revisa '
                          '"estado" y los parámetros montadas/estados')


def validar_arbol(cfg: ConfigBrazo, padres: Dict[str, Optional[str]]) -> None:
    """Todo lo que cuelga de una articulación ausente debe estar ausente.

    Args:
        padres: nombre -> nombre de la articulación padre (None en la raíz),
            sólo para articulaciones del perfil.
    """
    estado = {a.nombre: a.estado for a in cfg.articulaciones}
    for nom, est in estado.items():
        if est == 'ausente':
            continue
        p = padres.get(nom)
        while p is not None:
            if estado.get(p) == 'ausente':
                raise ErrorConfig(
                    f'{nom} está "{est}" pero cuelga de {p}, que está '
                    f'"ausente". Si {p} no está montada, {nom} tampoco: '
                    f'márcala "ausente".')
            p = padres.get(p)


def cargar(perfil: str, plataforma: str = '', ajustes: str = '',
           montadas: int = 0, estados: Optional[Dict[str, str]] = None,
           buscar_paquete=None) -> ConfigBrazo:
    """Lee los archivos de cada capa y construye la configuración."""
    capas, archivos = [], []
    for ruta in (perfil, plataforma, ajustes):
        if not ruta:
            continue
        if ruta == ajustes and not os.path.isfile(ruta):
            continue            # los ajustes guardados son opcionales
        capas.append(leer_yaml(ruta))
        archivos.append(ruta)
    if not capas:
        raise ErrorConfig('No se indicó ningún perfil de brazo')
    return construir(capas, archivos, montadas, estados, buscar_paquete)


def parsear_estados(texto: str) -> Dict[str, str]:
    """Convierte "Joint_2=ausente,Joint_3=bloqueada" en un dict."""
    out: Dict[str, str] = {}
    for parte in (texto or '').replace(';', ',').split(','):
        parte = parte.strip()
        if not parte:
            continue
        if '=' not in parte:
            raise ErrorConfig(f'estados: "{parte}" debe tener la forma '
                              f'Nombre=estado')
        nom, est = (s.strip() for s in parte.split('=', 1))
        _elegir(est, ESTADOS, f'estado de {nom}', 'estados')
        out[nom] = est
    return out
