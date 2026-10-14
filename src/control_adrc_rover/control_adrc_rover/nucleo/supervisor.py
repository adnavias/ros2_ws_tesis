"""Supervisor en operación y estimación en línea de kappa.

Supervisor. Al terminar cada movimiento se observa una ventana de
asentamiento y, por articulación, se mide:

    sobreimpulso = max_t ( (q - q_f) · signo(q_f - q_0) )      [rad]
    cruces       = cambios de signo del error fuera de una banda muerta.

Si el sobreimpulso supera max(umbral_deg, umbral_pct·|Δq|) o hay más de
`cruces_max` cruces, se escalan todos los polos de esa articulación por
`factor_reduccion` (~0.85) sin bajar de `wn_min`. El supervisor NUNCA sube
ganancias; para subirlas está el autotuning o la edición a mano.

Ojo: bajar wn sólo ayuda si la oscilación viene de un kappa alto (modelo
que sobreestima la inercia) o de retardos. Si kappa es bajo (modelo que
subestima la inercia), el amortiguamiento no depende de wn y el supervisor
lo avisa para que se recalibre (autotune) en lugar de seguir bajando.

Estimación de kappa. Con buen seguimiento q'' ≈ q*'' y v ≈ q*'' - phi_hat;
de q'' = kappa v + d se obtiene

    phi_hat ≈ (1 - 1/kappa) q*'' + d/kappa,

así que una regresión lineal de phi_hat contra q*'' (con término
constante para d lento) da la pendiente s y kappa = 1/(1 - s). Se usa
mínimos cuadrados recursivos con olvido y sólo se actualiza cuando la
aceleración de referencia es significativa.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np


@dataclass
class ResultadoMovimiento:
    """Métricas de un movimiento terminado (una articulación)."""

    nombre: str
    desplazamiento_deg: float
    sobreimpulso_deg: float
    sobreimpulso_pct: float
    cruces: int
    error_final_deg: float


def analizar(nombre: str, q: np.ndarray, q0: float, qf: float,
             banda: float) -> ResultadoMovimiento:
    """Métricas de la ventana de asentamiento de una articulación."""
    q = np.asarray(q, dtype=float)
    dq = qf - q0
    direccion = 1.0 if dq >= 0.0 else -1.0
    if q.size == 0:
        return ResultadoMovimiento(nombre, math.degrees(dq), 0.0, 0.0, 0, 0.0)
    exceso = float(np.max((q - qf) * direccion))
    sobre = max(exceso, 0.0) if abs(dq) > 1e-9 else float(
        np.max(np.abs(q - qf)))
    e = q - qf
    signos = np.sign(np.where(np.abs(e) > banda, e, 0.0))
    signos = signos[signos != 0.0]
    cruces = int(np.sum(signos[1:] != signos[:-1])) if signos.size > 1 else 0
    pct = 100.0 * sobre / abs(dq) if abs(dq) > 1e-9 else 0.0
    return ResultadoMovimiento(nombre, math.degrees(dq), math.degrees(sobre),
                               pct, cruces, math.degrees(float(e[-1])))


class Supervisor:
    """Decide reducciones de ganancia tras cada movimiento."""

    def __init__(self, cfg: dict) -> None:
        """cfg: sección `supervisor` de la configuración."""
        self.habilitado = bool(cfg['habilitado'])
        self.umbral_deg = float(cfg['umbral_sobreimpulso_deg'])
        self.umbral_pct = float(cfg['umbral_sobreimpulso_pct'])
        self.cruces_max = int(cfg['cruces_max'])
        self.factor = float(cfg['factor_reduccion'])
        self.wn_min = float(cfg['wn_min'])
        self.ventana = float(cfg['ventana'])
        self.banda = math.radians(float(cfg['banda_cruces_deg']))
        self.eventos: List[str] = []

    def excede(self, r: ResultadoMovimiento) -> bool:
        """True si el movimiento tuvo sobreimpulso u oscilación."""
        umbral = max(self.umbral_deg,
                     self.umbral_pct / 100.0 * abs(r.desplazamiento_deg))
        return r.sobreimpulso_deg > umbral or r.cruces > self.cruces_max

    def decidir(self, r: ResultadoMovimiento, wn_actual: float,
                kappa: Optional[float]) -> Optional[float]:
        """Factor de escala de polos (<1) o None si no hay que bajar."""
        if not self.habilitado or not self.excede(r):
            return None
        if kappa is not None and kappa < 0.6:
            self.eventos.append(
                f'{r.nombre}: sobreimpulso {r.sobreimpulso_deg:.2f}° / '
                f'{r.cruces} cruces con kappa estimado {kappa:.2f} < 0.6: el '
                f'modelo subestima la inercia; bajar wn no lo corrige. '
                f'Ejecuta el autotuning.')
            return None
        nuevo = max(self.wn_min, wn_actual * self.factor)
        if nuevo >= wn_actual - 1e-9:
            self.eventos.append(
                f'{r.nombre}: oscila pero wn ya está en el mínimo '
                f'({self.wn_min:.1f} rad/s); revisa la calibración.')
            return None
        self.eventos.append(
            f'{r.nombre}: sobreimpulso {r.sobreimpulso_deg:.2f}° '
            f'({r.sobreimpulso_pct:.1f} %), {r.cruces} cruces -> wn '
            f'{wn_actual:.2f} -> {nuevo:.2f} rad/s')
        return nuevo / wn_actual


class EstimadorKappa:
    """RLS de phi_hat = s q*'' + c por articulación; kappa = 1/(1-s)."""

    def __init__(self, n: int, olvido: float = 0.998,
                 umbral_acc: float = 0.2) -> None:
        """Args:
            olvido: factor de olvido λ (0.998 a 200 Hz ≈ 2.5 s de memoria).
            umbral_acc: |q*''| mínimo [rad/s^2] para actualizar.
        """
        self.n = n
        self.olvido = float(olvido)
        self.umbral = float(umbral_acc)
        self.reiniciar()

    def reiniciar(self) -> None:
        """Vuelve a la estimación inicial (kappa desconocido)."""
        self.theta = np.zeros((self.n, 2))
        self.p = np.tile(np.eye(2) * 100.0, (self.n, 1, 1))
        self.muestras = np.zeros(self.n, dtype=int)

    def actualizar(self, qdd_ref: np.ndarray, phi_hat: np.ndarray) -> None:
        """Una muestra por articulación."""
        for i in range(self.n):
            if abs(qdd_ref[i]) < self.umbral or not np.isfinite(phi_hat[i]):
                continue
            x = np.array([qdd_ref[i], 1.0])
            p = self.p[i]
            px = p @ x
            k = px / (self.olvido + x @ px)
            self.theta[i] += k * (phi_hat[i] - x @ self.theta[i])
            self.p[i] = (p - np.outer(k, px)) / self.olvido
            self.muestras[i] += 1

    def kappa(self, minimo_muestras: int = 100) -> np.ndarray:
        """kappa estimado (NaN si aún no hay datos suficientes)."""
        s = self.theta[:, 0]
        with np.errstate(divide='ignore', invalid='ignore'):
            k = 1.0 / (1.0 - s)
        k = np.where((self.muestras >= minimo_muestras) & (s < 0.95), k,
                     np.nan)
        return k
