"""Protecciones del lazo: sanidad del encoder, watchdog de medición,
paredes virtuales en los límites articulares, límite de pendiente del
mando y validación de objetivos.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

TWO_PI = 2.0 * math.pi


class SanidadEncoder:
    """Filtra NaN, saltos imposibles y vueltas de articulaciones continuas.

    - NaN o infinito: se rechaza la muestra (se conserva la anterior).
    - Articulación continua cuyo encoder da la vuelta (salta ~2π): se
      desenvuelve para que la posición sea continua.
    - Salto mayor que `salto_max` en una muestra: se rechaza.
    - Más de `invalidas_max` muestras seguidas rechazadas: falla.
    """

    def __init__(self, salto_max: np.ndarray, desenvolver: np.ndarray,
                 invalidas_max: int = 10) -> None:
        """Configura por articulación los umbrales y el desenvolvimiento."""
        self.salto_max = np.asarray(salto_max, dtype=float)
        self.desenvolver = np.asarray(desenvolver, dtype=bool)
        self.invalidas_max = int(invalidas_max)
        n = self.salto_max.size
        self.ultimo = np.full(n, np.nan)
        self.vueltas = np.zeros(n)
        self.seguidas = np.zeros(n, dtype=int)
        self.rechazos = np.zeros(n, dtype=int)
        self._crudo_prev = np.full(n, np.nan)

    def reiniciar(self) -> None:
        """Olvida la historia (la siguiente muestra se acepta tal cual)."""
        self.ultimo[:] = np.nan
        self.vueltas[:] = 0.0
        self.seguidas[:] = 0
        self._crudo_prev[:] = np.nan

    def procesar(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, bool]:
        """Devuelve (q_filtrada, muestra_valida, falla)."""
        q = np.asarray(q, dtype=float).copy()
        valida = np.isfinite(q)
        # desenvolvimiento de vueltas en articulaciones continuas
        prev = self._crudo_prev
        hay_prev = np.isfinite(prev) & valida
        d = np.where(hay_prev, q - prev, 0.0)
        salto_vuelta = self.desenvolver & hay_prev & (np.abs(d) > math.pi)
        self.vueltas -= np.where(salto_vuelta, np.round(d / TWO_PI), 0.0)
        self._crudo_prev = np.where(valida, q, prev)
        q = q + TWO_PI * self.vueltas
        # salto imposible
        primera = ~np.isfinite(self.ultimo)
        salto = valida & ~primera & (np.abs(q - self.ultimo)
                                     > self.salto_max)
        valida = valida & ~salto
        self.seguidas = np.where(valida, 0, self.seguidas + 1)
        self.rechazos += (~valida).astype(int)
        self.ultimo = np.where(valida, q, self.ultimo)
        falla = bool(np.any(self.seguidas > self.invalidas_max))
        return self.ultimo.copy(), valida, falla


class EstimadorVelocidad:
    """Derivada filtrada de la posición (paredes virtuales, diagnóstico)."""

    def __init__(self, n: int, fc_hz: float = 15.0) -> None:
        """Filtro de primer orden de frecuencia de corte fc_hz."""
        self.tau = 1.0 / (2.0 * math.pi * fc_hz)
        self.v = np.zeros(n)
        self._q = None

    def reiniciar(self) -> None:
        """Pone la velocidad en cero."""
        self.v[:] = 0.0
        self._q = None

    def actualizar(self, q: np.ndarray, dt: float) -> np.ndarray:
        """Nueva muestra de posición; devuelve la velocidad filtrada."""
        if self._q is None or dt <= 0.0:
            self._q = np.asarray(q, dtype=float).copy()
            return self.v
        cruda = (q - self._q) / dt
        a = dt / (self.tau + dt)
        self.v = self.v + a * (cruda - self.v)
        self._q = np.asarray(q, dtype=float).copy()
        return self.v


def estado_watchdog(edad: float, t_congelar: float, t_quitar: float) -> str:
    """'ok', 'congelar' o 'quitar' según la edad de la última medición."""
    if edad > t_quitar:
        return 'quitar'
    if edad > t_congelar:
        return 'congelar'
    return 'ok'


def aceleracion_pared(q: np.ndarray, qd: np.ndarray, lo: np.ndarray,
                      hi: np.ndarray, margen: np.ndarray, k: np.ndarray,
                      d: np.ndarray) -> np.ndarray:
    """Aceleración de las paredes virtuales [rad/s^2].

    Dentro de [lo + margen, hi - margen] vale cero. Fuera, es un resorte
    proporcional a la penetración más un amortiguador que sólo actúa
    mientras la articulación se mete en la pared.
    """
    a = np.zeros_like(q)
    lim_lo = lo + margen
    lim_hi = hi - margen
    pen_lo = np.where(np.isfinite(lim_lo), lim_lo - q, -1.0)
    pen_hi = np.where(np.isfinite(lim_hi), q - lim_hi, -1.0)
    entra_lo = (pen_lo > 0.0)
    entra_hi = (pen_hi > 0.0)
    a = np.where(entra_lo, k * pen_lo - d * np.minimum(qd, 0.0), a)
    a = np.where(entra_hi, -k * pen_hi - d * np.maximum(qd, 0.0), a)
    return a


class LimitadorPendiente:
    """Limita la variación del mando por segundo (0 = sin límite)."""

    def __init__(self, pendiente_max: np.ndarray) -> None:
        """Pendiente máxima por articulación (unidades del mando / s)."""
        self.pendiente = np.asarray(pendiente_max, dtype=float)
        self.previo = None

    def reiniciar(self, valor: np.ndarray = None) -> None:
        """Olvida el último mando (o lo fija en `valor`)."""
        self.previo = None if valor is None else np.asarray(valor).copy()

    def aplicar(self, cmd: np.ndarray, dt: float) -> np.ndarray:
        """Recorta la variación respecto al mando anterior."""
        cmd = np.asarray(cmd, dtype=float)
        if self.previo is None:
            self.previo = cmd.copy()
            return cmd
        con = self.pendiente > 0.0
        paso = self.pendiente * max(dt, 0.0)
        out = np.where(con, np.clip(cmd, self.previo - paso,
                                    self.previo + paso), cmd)
        self.previo = out.copy()
        return out


def validar_objetivo(objetivo: np.ndarray, lo: np.ndarray, hi: np.ndarray,
                     margen: np.ndarray, modo: str, nombres: List[str]
                     ) -> Tuple[np.ndarray, List[str]]:
    """Recorta (o rechaza) un objetivo fuera de los límites del URDF.

    Returns:
        (objetivo_valido, avisos). Con modo 'rechazar' lanza ValueError.
    """
    lim_lo = lo + margen
    lim_hi = hi - margen
    fuera_lo = np.isfinite(lim_lo) & (objetivo < lim_lo)
    fuera_hi = np.isfinite(lim_hi) & (objetivo > lim_hi)
    avisos = []
    for i in np.nonzero(fuera_lo | fuera_hi)[0]:
        lim = lim_lo[i] if fuera_lo[i] else lim_hi[i]
        avisos.append(f'{nombres[i]}: {math.degrees(objetivo[i]):.1f}° fuera '
                      f'de rango; límite {math.degrees(lim):.1f}° (URDF)')
    if avisos and modo == 'rechazar':
        raise ValueError('Objetivo rechazado: ' + '; '.join(avisos))
    return np.clip(objetivo, np.where(np.isfinite(lim_lo), lim_lo, -np.inf),
                   np.where(np.isfinite(lim_hi), lim_hi, np.inf)), avisos
