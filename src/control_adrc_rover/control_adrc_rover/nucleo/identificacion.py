"""Identificación por mínimos cuadrados de la inercia, la armadura y la
fricción de una articulación, y elección de beta.

Durante un movimiento seguido con buen error (|e| pequeño) la aceleración
real es la de referencia, q'' ≈ q*''; así se evita derivar el encoder dos
veces. Para la articulación i, con las demás quietas:

    K_hat i_i = alfa M_ii(q) q*''_i + J_a q*''_i
                + f_c σ_i + alfa_g g_i(q)

con σ_i el regresor de fricción: tanh del mando (modelo "zona_muerta",
igual que actuador_sim) o tanh(q*'/0.02) (modelo "velocidad").

donde M_ii y g_i son los del URDF (con las articulaciones ausentes ya
quitadas) e i_i es la corriente mandada SIN la compensación de fricción
que suma el control (ésa se descuenta antes). Las incógnitas son

    θ = [alfa, J_a, f_c, alfa_g].

alfa y J_a sólo se separan si M_ii cambia entre las posturas (J1 con el
hombro, J2 con el codo). Si no cambia lo suficiente se identifica un solo
factor c de la inercia total, K_hat i = c (M_ii + J_a,nom) q*'' + ..., y se
reporta alfa = c, J_a = c J_a,nom (se informa como "armadura fijada").

Como la corriente se multiplica por K_hat, cualquier error de K_tau queda
absorbido en alfa, J_a, f_c y alfa_g: el modelo calibrado es el que el
lazo realmente "ve", que es lo que importa para kappa.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

import numpy as np


@dataclass
class Muestras:
    """Datos de una articulación acumulados durante la identificación."""

    tau: List[float] = field(default_factory=list)     # K_hat * i [N·m]
    qdd: List[float] = field(default_factory=list)     # q*'' [rad/s^2]
    fric: List[float] = field(default_factory=list)    # regresor de fricción
    m_ii: List[float] = field(default_factory=list)    # M_ii(q) URDF
    g_i: List[float] = field(default_factory=list)     # g_i(q) URDF

    def agregar(self, tau, qdd, fric, m_ii, g_i) -> None:
        """Agrega una muestra (fric = regresor en [-1, 1])."""
        self.tau.append(float(tau))
        self.qdd.append(float(qdd))
        self.fric.append(float(fric))
        self.m_ii.append(float(m_ii))
        self.g_i.append(float(g_i))

    def __len__(self) -> int:
        return len(self.tau)


@dataclass
class ResultadoIdentificacion:
    """Parámetros identificados de una articulación."""

    alfa: float
    armadura: float
    friccion_tau: float        # [N·m]
    alfa_gravedad: float
    sigma_alfa: float          # desviación estándar estimada de alfa
    residuo_rms: float         # [N·m]
    armadura_fijada: bool
    n_muestras: int
    mensaje: str = ''


def identificar(m: Muestras,
                armadura_nominal: float) -> ResultadoIdentificacion:
    """Mínimos cuadrados de θ = [alfa, J_a, f_c, alfa_g]."""
    if len(m) < 20:
        raise ValueError(f'Muy pocas muestras para identificar ({len(m)})')
    tau = np.asarray(m.tau)
    qdd = np.asarray(m.qdd)
    mii = np.asarray(m.m_ii)
    gi = np.asarray(m.g_i)
    fric = np.asarray(m.fric)

    variacion = np.std(mii) / max(np.mean(np.abs(mii)), 1e-9)
    excitacion_g = np.std(gi) > 1e-3 or np.mean(np.abs(gi)) > 1e-3
    fijada = variacion < 0.05
    if fijada:
        # alfa escala la inercia total (eslabones + armadura nominal)
        columnas = [(mii + armadura_nominal) * qdd]
    else:
        columnas = [mii * qdd, qdd]
    nombres = ['alfa'] if fijada else ['alfa', 'armadura']
    columnas.append(fric)
    nombres.append('friccion')
    y = tau.copy()
    if excitacion_g:
        columnas.append(gi)
        nombres.append('alfa_g')
    a = np.column_stack(columnas)
    theta, *_ = np.linalg.lstsq(a, y, rcond=None)
    residuo = y - a @ theta
    dof = max(len(y) - a.shape[1], 1)
    s2 = float(residuo @ residuo) / dof
    try:
        cov = s2 * np.linalg.inv(a.T @ a)
        sigmas = np.sqrt(np.clip(np.diag(cov), 0.0, None))
    except np.linalg.LinAlgError:
        sigmas = np.full(a.shape[1], np.inf)
    val = dict(zip(nombres, theta))
    sig = dict(zip(nombres, sigmas))
    alfa = float(val['alfa'])
    armadura = (alfa * armadura_nominal if fijada
                else float(val.get('armadura', armadura_nominal)))
    mensaje = []
    if fijada:
        mensaje.append('M_ii casi constante entre posturas: no se separan '
                       'eslabón y rotor; alfa escala la inercia total '
                       '(armadura = alfa * nominal)')
    if armadura < 0.0:
        mensaje.append(f'armadura identificada negativa ({armadura:.3f}); '
                       f'se usa la nominal')
        armadura = armadura_nominal
    if alfa <= 0.05:
        raise ValueError(f'alfa identificado no físico ({alfa:.3f}); revisa '
                         f'el signo de la articulación y los datos')
    return ResultadoIdentificacion(
        alfa=alfa, armadura=armadura,
        friccion_tau=max(float(val['friccion']), 0.0),
        alfa_gravedad=float(val.get('alfa_g', 1.0)),
        sigma_alfa=float(sig['alfa']),
        residuo_rms=math.sqrt(s2), armadura_fijada=fijada,
        n_muestras=len(y), mensaje='; '.join(mensaje))


def elegir_beta(alfa: float, sigma_alfa: float, k_objetivo: float = 1.0,
                beta_min: float = 0.6) -> float:
    """beta para que kappa ≲ k_objetivo aun con la incertidumbre de alfa.

    Tras calibrar, kappa nominal es 1. Con alfa incierto (±2σ) el peor
    kappa es (alfa + 2σ)/alfa; beta lo devuelve a k_objetivo. No se baja de
    beta_min porque un kappa bajo también degrada el amortiguamiento.
    """
    peor = (alfa + 2.0 * abs(sigma_alfa)) / alfa
    return float(np.clip(k_objetivo / peor, beta_min, 1.0))


def kappa_en_rejilla(m_control, m_real, posturas) -> np.ndarray:
    """kappa_i = M_control,ii / M_real,ii sobre una lista de posturas.

    Args:
        m_control, m_real: funciones q -> matriz M.
    Returns:
        arreglo (n_posturas, n) con kappa por articulación.
    """
    filas = []
    for q in posturas:
        mc = np.diag(m_control(q))
        mr = np.diag(m_real(q))
        filas.append(mc / np.maximum(mr, 1e-12))
    return np.array(filas)
