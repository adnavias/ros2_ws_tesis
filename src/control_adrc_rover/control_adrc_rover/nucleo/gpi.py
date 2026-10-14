"""Compensador GPI (Sira-Ramírez) por articulación, con polos reales
configurables, anti-windup y estimación implícita de la perturbación.

Cada articulación controlada se trata como el doble integrador perturbado

    q'' = kappa v + phi,      kappa = M_ctrl / M_real (ideal 1),

con error e = q - q*. El compensador tiene dos estados integrales:

    s2' = e - k3 s2,     s1' = s2,
    v   = q*'' - ( k2 s2' + k1 s2 + k0 s1 ).

En lazo cerrado (kappa = 1, phi constante) el error cumple

    p(s) = s^4 + k3 s^3 + k2 s^2 + k1 s + k0 = (s+λ1)(s+λ2)(s+λ3)(s+λ4),

así que los cuatro polos -λi se eligen reales y negativos y las ganancias
salen de expandir el producto:

    k3 = Σ λi,   k2 = Σ_{i<j} λiλj,   k1 = Σ_{i<j<k} λiλjλk,   k0 = Π λi.

Con kappa ≠ 1 el polinomio es  s^3 (s+k3) + kappa (k2 s^2 + k1 s + k0);
con la medición retrasada 1–2 muestras a 200 Hz el lazo es estable para
0.15 ≲ kappa ≲ 2.3 y bien amortiguado sólo para 0.6 ≲ kappa ≲ 1.4 (ver la
guía). Por eso la calibración del modelo (alfa, J_a) y beta importan.

Perturbación estimada. En régimen (e → 0) la parte integral del mando
cancela la perturbación; de  v - q*'' = (k2 k3 - k1) s2 - k0 s1 = -phi  se
obtiene la estimación implícita

    phi_hat = k0 s1 + (k1 - k2 k3) s2      [rad/s^2],

que se publica para diagnóstico y para estimar modos de vibración.

Anti-windup (back-calculation). Si el mando satura, la diferencia
Δv = v_sat - v (en aceleración) se realimenta al integrador s1 con
ganancia k_aw, y s2 se congela si su derivada empuja más hacia la
saturación.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np


def ganancias_desde_polos(polos: Sequence[float]):
    """(k3, k2, k1, k0) del producto Π (s + λi) con λi > 0."""
    c = np.poly(-np.asarray(polos, dtype=float))
    return float(c[1]), float(c[2]), float(c[3]), float(c[4])


class GPI:
    """Banco de compensadores GPI independientes (uno por articulación)."""

    def __init__(self, polos: np.ndarray, k_aw: float = 1.0) -> None:
        """Args:
            polos: arreglo (m, 4) con los λi (> 0) de cada articulación.
            k_aw: ganancia de anti-windup, relativa a λ1 de cada una.
        """
        polos = np.atleast_2d(np.asarray(polos, dtype=float))
        self.m = polos.shape[0]
        self.polos = np.zeros((self.m, 4))
        self.k3 = np.zeros(self.m)
        self.k2 = np.zeros(self.m)
        self.k1 = np.zeros(self.m)
        self.k0 = np.zeros(self.m)
        self.k_aw = float(k_aw)
        for i in range(self.m):
            self.fijar_polos(i, polos[i])
        self.s1 = np.zeros(self.m)
        self.s2 = np.zeros(self.m)

    # ------------------------------------------------------------------
    def fijar_polos(self, i: int, polos: Sequence[float]) -> None:
        """Cambia los polos de la articulación i (conserva el mando)."""
        polos = np.asarray(polos, dtype=float)
        if polos.shape != (4,) or np.any(polos <= 0.0):
            raise ValueError('Se necesitan 4 polos positivos')
        # Conserva la contribución integral para no dar un salto de mando.
        phi_prev = None
        if hasattr(self, 's1'):
            phi_prev = self.phi_hat()[i]
        self.polos[i] = polos
        self.k3[i], self.k2[i], self.k1[i], self.k0[i] = (
            ganancias_desde_polos(polos))
        if phi_prev is not None:
            self.s2[i] = 0.0
            self.s1[i] = phi_prev / self.k0[i]

    def escalar(self, i: int, factor: float) -> None:
        """Escala todos los polos de la articulación i (p. ej. 0.85)."""
        self.fijar_polos(i, self.polos[i] * float(factor))

    def wn(self) -> np.ndarray:
        """Polo más lento de cada articulación (≈ ancho de banda)."""
        return self.polos.min(axis=1)

    # ------------------------------------------------------------------
    def reiniciar(self, v0: Optional[np.ndarray] = None) -> None:
        """Pone los integradores para que, con e = 0, el mando sea v0."""
        self.s2[:] = 0.0
        self.s1[:] = 0.0 if v0 is None else -np.asarray(v0) / self.k0

    def calcular(self, e: np.ndarray, qdd_ref: np.ndarray):
        """Mando virtual v y derivada de s2 (no modifica el estado)."""
        s2_dot = e - self.k3 * self.s2
        v = qdd_ref - (self.k2 * s2_dot + self.k1 * self.s2
                       + self.k0 * self.s1)
        return v, s2_dot

    def parte_integral(self) -> np.ndarray:
        """Aporte de los integradores a v con e = 0 (= -phi_hat)."""
        return (self.k2 * self.k3 - self.k1) * self.s2 - self.k0 * self.s1

    def phi_hat(self) -> np.ndarray:
        """Perturbación estimada [rad/s^2]."""
        return -self.parte_integral()

    def integrar(self, s2_dot: np.ndarray, dt: float,
                 congelar: Optional[np.ndarray] = None,
                 delta_v: Optional[np.ndarray] = None) -> None:
        """Avanza los integradores con Euler.

        Args:
            congelar: máscara de articulaciones cuyos integradores no se
                mueven (watchdog, paro).
            delta_v: v_sat - v por saturación (0 si no satura).
        """
        s2_dot = np.asarray(s2_dot, dtype=float)
        avanza = np.ones(self.m, dtype=bool) if congelar is None else ~congelar
        ds1 = self.s2 * dt
        ds2 = s2_dot * dt
        if delta_v is not None:
            dv = np.asarray(delta_v, dtype=float)
            sat = np.abs(dv) > 1e-12
            # back-calculation en s1: mueve v hacia v_sat
            ds1 = ds1 - np.where(sat, self.k_aw * self.polos[:, 0] * dt
                                 * dv / self.k0, 0.0)
            # integración condicional de s2: no empujar más la saturación
            empuja = (self.k2 * self.k3 - self.k1) * ds2 * (-dv) > 0.0
            ds2 = np.where(sat & empuja, 0.0, ds2)
        self.s1 = np.where(avanza, self.s1 + ds1, self.s1)
        self.s2 = np.where(avanza, self.s2 + ds2, self.s2)

    def ajustar_s1(self, delta_v_integral: np.ndarray) -> None:
        """Suma delta a la parte integral de v (traspaso de gravedad)."""
        self.s1 = self.s1 - np.asarray(delta_v_integral) / self.k0
