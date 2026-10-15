"""Compensador GPI (Sira-Ramírez) por articulación, sintonizable por polos
o por (ζ, wn, ε), anti-windup y estimación implícita de la perturbación.

Cada articulación controlada se trata como el doble integrador perturbado

    q'' = kappa v + phi,      kappa = M_ctrl / M_real (ideal 1),

con error e = q - q*. El compensador tiene dos estados integrales:

    s2' = e - k3 s2,     s1' = s2,
    v   = q*'' - ( k2 s2' + k1 s2 + k0 s1 ).

En lazo cerrado (kappa = 1, phi constante) el error cumple

    p(s) = s^4 + k3 s^3 + k2 s^2 + k1 s + k0 = (s+λ1)(s+λ2)(s+λ3)(s+λ4),

y las ganancias salen de expandir el producto. Hay dos formas de elegir
p(s) (la "sintonía" de cada articulación):

  * modo "polos": los cuatro λi reales y positivos,
        k3 = Σ λi,  k2 = Σ_{i<j} λiλj,  k1 = Σ_{i<j<k} λiλjλk,  k0 = Π λi.
  * modo "zeta": dos pares de segundo orden,
        p(s) = (s^2 + 2 ζ wn s + wn^2) (s^2 + 2 ζo wp s + wp^2),
        wp = wn / ε,   ζo = ζ salvo que se indique otro,
    el par lento (ζ, wn) fija cómo converge el error y el par rápido
    (ζo, wp) es la parte "observador" que estima la perturbación; ε < 1
    dice cuántas veces más rápido es el observador. Con ζ ≥ 1 los cuatro
    polos son reales (sin sobreimpulso del error); con ζ < 1 son
    complejos y el error puede sobrepasar. ζ = 1 equivale exactamente a
    polos [wn, wn, wn/ε, wn/ε] (p. ej. [8, 8, 32, 32] = wn 8, ζ 1, ε 0.25).

Cualquier juego de 4 polos reales se puede escribir también como (wn, ζ,
ε, ζo) agrupando los dos lentos y los dos rápidos, así que la interfaz
muestra siempre las dos vistas.

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

import math
from dataclasses import dataclass, replace
from typing import Optional, Sequence, Tuple

import numpy as np

MODOS_SINTONIA = ('polos', 'zeta')
ZETA_MIN = 0.2
ZETA_MAX = 5.0


def ganancias_desde_polos(polos: Sequence[float]):
    """(k3, k2, k1, k0) del producto Π (s + λi) con λi > 0."""
    c = np.poly(-np.asarray(polos, dtype=float))
    return float(c[1]), float(c[2]), float(c[3]), float(c[4])


def _par(zeta: float, w: float) -> np.ndarray:
    """Coeficientes de s^2 + 2 ζ w s + w^2."""
    return np.array([1.0, 2.0 * zeta * w, w * w])


def _raices_par(zeta: float, w: float) -> np.ndarray:
    """Raíces de s^2 + 2 ζ w s + w^2, analíticas (ζ = 1 da raíz doble
    exactamente real, sin el ruido numérico de np.roots)."""
    d = zeta * zeta - 1.0
    if d >= 0.0:
        r = math.sqrt(d)
        return np.array([-w * (zeta - r), -w * (zeta + r)], dtype=complex)
    im = w * math.sqrt(-d)
    return np.array([complex(-zeta * w, im), complex(-zeta * w, -im)])


def _par_desde_reales(a: float, b: float) -> Tuple[float, float]:
    """(w, ζ) del par (s + a)(s + b)."""
    w = math.sqrt(a * b)
    return w, (a + b) / (2.0 * w)


@dataclass(frozen=True)
class Sintonia:
    """Cómo se eligen los 4 polos del GPI de una articulación.

    modo "polos": usa `polos` (λ1..λ4 > 0).
    modo "zeta": usa wn, zeta, eps y zeta_observador (<= 0 -> igual a
    zeta). Los campos del otro modo se guardan como vista equivalente
    (ver `completa`), sólo para mostrarlos.
    """

    modo: str = 'polos'
    polos: Tuple[float, float, float, float] = (8.0, 8.0, 32.0, 32.0)
    wn: float = 8.0
    zeta: float = 1.0
    eps: float = 0.25
    zeta_observador: float = 0.0

    # -- construcción ---------------------------------------------------
    @staticmethod
    def desde_polos(polos: Sequence[float]) -> 'Sintonia':
        """Sintonía en modo polos (con la vista ζ equivalente)."""
        p = tuple(float(x) for x in polos)
        if len(p) != 4:
            raise ValueError('se necesitan 4 polos')
        return Sintonia(modo='polos', polos=p).completa()

    @staticmethod
    def desde_zeta(wn: float, zeta: float, eps: float,
                   zeta_observador: float = 0.0) -> 'Sintonia':
        """Sintonía en modo zeta (con la vista de polos equivalente)."""
        return Sintonia(modo='zeta', wn=float(wn), zeta=float(zeta),
                        eps=float(eps),
                        zeta_observador=float(zeta_observador)).completa()

    @property
    def zeta_obs(self) -> float:
        """ζ del par rápido (observador)."""
        return self.zeta if self.zeta_observador <= 0.0 else \
            self.zeta_observador

    # -- polinomio --------------------------------------------------------
    def polinomio(self) -> np.ndarray:
        """Coeficientes [1, k3, k2, k1, k0] de p(s)."""
        if self.modo == 'polos':
            return np.poly(-np.asarray(self.polos, dtype=float))
        wp = self.wn / self.eps
        return np.polymul(_par(self.zeta, self.wn), _par(self.zeta_obs, wp))

    def coeficientes(self) -> Tuple[float, float, float, float]:
        """(k3, k2, k1, k0)."""
        c = np.real(self.polinomio())
        return float(c[1]), float(c[2]), float(c[3]), float(c[4])

    def raices(self) -> np.ndarray:
        """Raíces de p(s) (parte real negativa), ordenadas por |s|."""
        if self.modo == 'polos':
            r = -np.asarray(self.polos, dtype=complex)
        else:
            r = np.concatenate([
                _raices_par(self.zeta, self.wn),
                _raices_par(self.zeta_obs, self.wn / self.eps)])
        return r[np.lexsort((r.imag, np.abs(r)))]

    def wn_efectivo(self) -> float:
        """Ancho de banda de referencia (λ mínimo o wn)."""
        return float(min(self.polos)) if self.modo == 'polos' else \
            float(self.wn)

    # -- vistas y escalado -----------------------------------------------
    def completa(self) -> 'Sintonia':
        """Llena los campos del otro modo con la vista equivalente."""
        if self.modo == 'polos':
            p = sorted(self.polos)
            if p[0] <= 0.0:
                return self
            w1, z1 = _par_desde_reales(p[0], p[1])
            w2, z2 = _par_desde_reales(p[2], p[3])
            zo = 0.0 if abs(z2 - z1) < 1e-9 else z2
            return replace(self, wn=w1, zeta=z1, eps=w1 / w2,
                           zeta_observador=zo)
        if self.wn <= 0.0 or self.eps <= 0.0:
            return self
        r = self.raices()
        return replace(self, polos=tuple(float(x) for x in -r.real))

    def escalada(self, factor: float) -> 'Sintonia':
        """Multiplica todos los polos (o wn) por `factor`."""
        f = float(factor)
        if self.modo == 'polos':
            return Sintonia.desde_polos([p * f for p in self.polos])
        return Sintonia.desde_zeta(self.wn * f, self.zeta, self.eps,
                                   self.zeta_observador)

    def con_wn(self, wn: float) -> 'Sintonia':
        """Misma forma, con el ancho de banda de referencia = wn."""
        return self.escalada(float(wn) / self.wn_efectivo())

    def polos_reales(self) -> bool:
        """True si los 4 polos son reales (sin sobreimpulso del error)."""
        return bool(np.all(np.abs(self.raices().imag) < 1e-9))

    # -- validación -------------------------------------------------------
    def validar(self, dt: Optional[float] = None,
                wn_min: float = 0.0) -> None:
        """ValueError con un mensaje claro si la sintonía no sirve."""
        if self.modo not in MODOS_SINTONIA:
            raise ValueError(f'sintonia="{self.modo}" no válida; usa '
                             f'{list(MODOS_SINTONIA)}')
        if self.modo == 'polos':
            if len(self.polos) != 4 or any(not p > 0.0 for p in self.polos):
                raise ValueError(f'se necesitan 4 polos positivos (son -λ '
                                 f'en el plano s); llegaron {list(self.polos)}')
        else:
            if not self.wn > 0.0:
                raise ValueError('wn debe ser > 0')
            if not 0.0 < self.eps <= 1.0:
                raise ValueError(f'eps={self.eps} fuera de (0, 1]: el '
                                 f'observador debe ser igual o más rápido '
                                 f'que el lazo (wp = wn/eps)')
            for nom, z in (('zeta', self.zeta),
                           ('zeta_observador', self.zeta_obs)):
                if not ZETA_MIN <= z <= ZETA_MAX:
                    raise ValueError(f'{nom}={z} fuera de [{ZETA_MIN}, '
                                     f'{ZETA_MAX}]')
        if self.wn_efectivo() < wn_min:
            raise ValueError(f'el ancho de banda ({self.wn_efectivo():.2f} '
                             f'rad/s) debe ser >= {wn_min}')
        if dt is not None:
            rapido = float(np.max(np.abs(self.raices())))
            if rapido * dt > 0.5:
                raise ValueError(
                    f'el polo más rápido ({rapido:.1f} rad/s) da |λ|·dt = '
                    f'{rapido * dt:.2f} > 0.5 a {1.0 / dt:.0f} Hz; baja wn, '
                    f'sube eps o baja los polos')

    def advertencias(self) -> list:
        """Avisos no fatales (ζ < 0.7 -> sobreimpulso del error)."""
        avisos = []
        z = min(self.completa().zeta, self.completa().zeta_obs)
        if z < 0.7:
            avisos.append(f'ζ={z:.2f} < 0.7: polos complejos poco '
                          f'amortiguados; el error de seguimiento puede '
                          f'oscilar. Lo recomendado es ζ >= 0.9.')
        return avisos

    def describir(self) -> str:
        """Texto corto para logs."""
        if self.modo == 'polos':
            return 'polos [' + ', '.join(f'{p:.2f}' for p in self.polos) + ']'
        return (f'wn {self.wn:.2f}, ζ {self.zeta:.2f}, ε {self.eps:.3f}'
                + (f', ζo {self.zeta_observador:.2f}'
                   if self.zeta_observador > 0.0 else ''))

    def a_dict(self) -> dict:
        """Lo que se guarda en el YAML (sólo los campos de su modo)."""
        if self.modo == 'polos':
            return {'sintonia': 'polos',
                    'polos': [round(float(p), 4) for p in self.polos]}
        d = {'sintonia': 'zeta', 'wn': round(float(self.wn), 4),
             'zeta': round(float(self.zeta), 4),
             'eps': round(float(self.eps), 4)}
        if self.zeta_observador > 0.0:
            d['zeta_observador'] = round(float(self.zeta_observador), 4)
        return d


def _como_sintonia(x) -> Sintonia:
    if isinstance(x, Sintonia):
        return x.completa()
    return Sintonia.desde_polos(x)


class GPI:
    """Banco de compensadores GPI independientes (uno por articulación)."""

    def __init__(self, sintonias, k_aw: float = 1.0) -> None:
        """Args:
            sintonias: lista de `Sintonia` (una por articulación) o arreglo
                (m, 4) con los λi (> 0) de cada articulación.
            k_aw: ganancia de anti-windup, relativa al wn de cada una.
        """
        if isinstance(sintonias, np.ndarray):
            sintonias = np.atleast_2d(sintonias)
        lista = [_como_sintonia(s) for s in sintonias]
        self.m = len(lista)
        self.sintonias = list(lista)
        self.k3 = np.zeros(self.m)
        self.k2 = np.zeros(self.m)
        self.k1 = np.zeros(self.m)
        self.k0 = np.zeros(self.m)
        self._wn = np.zeros(self.m)
        self.k_aw = float(k_aw)
        for i, s in enumerate(lista):
            self.fijar_sintonia(i, s)
        self.s1 = np.zeros(self.m)
        self.s2 = np.zeros(self.m)

    # ------------------------------------------------------------------
    def fijar_sintonia(self, i: int, sintonia: Sintonia) -> None:
        """Cambia la sintonía de la articulación i (conserva el mando)."""
        s = _como_sintonia(sintonia)
        s.validar()
        # Conserva la contribución integral para no dar un salto de mando.
        phi_prev = None
        if hasattr(self, 's1'):
            phi_prev = self.phi_hat()[i]
        self.sintonias[i] = s
        self._wn[i] = s.wn_efectivo()
        self.k3[i], self.k2[i], self.k1[i], self.k0[i] = s.coeficientes()
        if phi_prev is not None:
            self.s2[i] = 0.0
            self.s1[i] = phi_prev / self.k0[i]

    def fijar_polos(self, i: int, polos: Sequence[float]) -> None:
        """Cambia a modo polos con los λi dados."""
        self.fijar_sintonia(i, Sintonia.desde_polos(polos))

    def escalar(self, i: int, factor: float) -> None:
        """Escala todos los polos de la articulación i (p. ej. 0.85)."""
        self.fijar_sintonia(i, self.sintonias[i].escalada(factor))

    def fijar_wn(self, i: int, wn: float) -> None:
        """Lleva el ancho de banda de la articulación i a wn (misma forma)."""
        self.fijar_sintonia(i, self.sintonias[i].con_wn(wn))

    def wn(self) -> np.ndarray:
        """Ancho de banda de referencia de cada articulación."""
        return self._wn.copy()

    @property
    def polos(self) -> np.ndarray:
        """(m, 4) partes reales -Re(s) de los polos (= λi si son reales)."""
        if self.m == 0:
            return np.zeros((0, 4))
        return np.array([-s.raices().real for s in self.sintonias])

    @property
    def polos_imag(self) -> np.ndarray:
        """(m, 4) partes imaginarias de los polos (0 si son reales)."""
        if self.m == 0:
            return np.zeros((0, 4))
        return np.array([s.raices().imag for s in self.sintonias])

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
            ds1 = ds1 - np.where(sat, self.k_aw * self._wn * dt
                                 * dv / self.k0, 0.0)
            # integración condicional de s2: no empujar más la saturación
            empuja = (self.k2 * self.k3 - self.k1) * ds2 * (-dv) > 0.0
            ds2 = np.where(sat & empuja, 0.0, ds2)
        self.s1 = np.where(avanza, self.s1 + ds1, self.s1)
        self.s2 = np.where(avanza, self.s2 + ds2, self.s2)

    def ajustar_s1(self, delta_v_integral: np.ndarray) -> None:
        """Suma delta a la parte integral de v (traspaso de gravedad)."""
        self.s1 = self.s1 - np.asarray(delta_v_integral) / self.k0
