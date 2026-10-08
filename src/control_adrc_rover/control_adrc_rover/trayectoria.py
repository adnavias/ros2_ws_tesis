"""Generador de trayectorias multiarticulares de 9.º grado (mínimo snap-dot).

Para cada articulación se construye el polinomio de 9.º grado

    q*(t) = sum_{k=0}^{9} a_k t^k,   t in [0, T],

con diez condiciones de frontera (posición y sus primeras cuatro derivadas
en ambos extremos):

    q*(0) = q0,  q*'(0) = v0,  q*''(0) = acc0,  q*'''(0) = j0,  q*''''(0) = s0,
    q*(T) = qf,  q*'(T) = 0,   q*''(T) = 0,     q*'''(T) = 0,   q*''''(T) = 0.

Con condiciones iniciales nulas se reduce a q0 + (qf - q0) s(tau), tau = t/T,

    s(tau) = 126 tau^5 - 420 tau^6 + 540 tau^7 - 315 tau^8 + 70 tau^9,

equivalente a una curva de Bézier de grado 9 con puntos de control
(q0, q0, q0, q0, q0, qf, qf, qf, qf, qf). Frente a la quíntica, la
referencia es continua hasta el snap (4.ª derivada) y el jerk arranca y
termina en cero; el contenido espectral de la aceleración, y por tanto del
torque de feedforward, decae más rápido en alta frecuencia, lo que reduce
la excitación de modos flexibles y holguras de la transmisión.

Replaneación a mitad de movimiento: si llega un objetivo nuevo mientras la
trayectoria está activa, el jerk y el snap iniciales se toman de la propia
trayectoria en curso (salvo que se den explícitamente), de modo que la
nueva curva no tiene saltos hasta la 4.ª derivada.

Duración automática y sincronizada: para un movimiento reposo-reposo,
    |q*'|_max  = 2.4609 |dq| / T        (315/128)
    |q*''|_max = 9.3720 |dq| / T^2,
de modo que la duración mínima que respeta V_MAX y A_MAX en la articulación
j es  T_j = max(2.4609 |dq_j| / V_MAX_j, sqrt(9.3720 |dq_j| / A_MAX_j)).
Todas las articulaciones usan T = max(T_min, max_j T_j / p), donde p es la
escala de velocidad (1.0 = 100 %). A igual desplazamiento y límites, el
movimiento dura ~31 % más que con la quíntica (cota de velocidad) o ~27 %
más (cota de aceleración): es el precio de la mayor suavidad.

Los coeficientes se calculan en tiempo normalizado tau in [0, 1] para
evitar el mal condicionamiento de las potencias altas de T.

El módulo es independiente de ROS: se puede probar y graficar por separado.

Autor: Adán Medina Covarrubias
"""

from math import factorial
from typing import Optional, Tuple

import numpy as np

ORDEN: int = 9
N_COEF: int = ORDEN + 1

PEAK_VEL_FACTOR: float = 315.0 / 128.0          # 2.4609375
PEAK_ACC_FACTOR: float = 9.371973879            # max |s''(tau)|, tau ~ 0.3110


def _matriz_final() -> np.ndarray:
    """Matriz 5x5 que liga b5..b9 con s^(m)(1), m = 0..4 (tau normalizado)."""
    A = np.zeros((5, 5))
    for m in range(5):
        for i, k in enumerate(range(5, N_COEF)):
            A[m, i] = factorial(k) / factorial(k - m)
    return A


_A_FIN_INV: np.ndarray = np.linalg.inv(_matriz_final())


class TrayectoriaNonica:
    """Trayectoria de 9.º grado sincronizada para N articulaciones."""

    def __init__(self, v_max: np.ndarray, a_max: np.ndarray,
                 t_min: float) -> None:
        """Guarda los límites y deja la trayectoria inactiva.

        Args:
            v_max: velocidad máxima por articulación [rad/s].
            a_max: aceleración máxima por articulación [rad/s^2].
            t_min: duración mínima de cualquier movimiento [s].
        """
        self.v_max = np.asarray(v_max, dtype=float)
        self.a_max = np.asarray(a_max, dtype=float)
        self.t_min = float(t_min)
        n = self.v_max.size
        self.coef = np.zeros((N_COEF, n))   # filas: b0..b9 en tau normalizado
        self.q_final = np.zeros(n)
        self.t0 = 0.0
        self.duration = 0.0
        self.active = False

    # ------------------------------------------------------------------
    def duracion(self, delta_q: np.ndarray, escala: float = 1.0) -> float:
        """Duración sincronizada que respeta escala*V_MAX y escala^2*A_MAX."""
        dq = np.abs(delta_q)
        t_vel = PEAK_VEL_FACTOR * dq / self.v_max
        t_acc = np.sqrt(PEAK_ACC_FACTOR * dq / self.a_max)
        t_req = max(float(np.max(t_vel)), float(np.max(t_acc))) / escala
        return max(self.t_min, t_req)

    def planear(self, t_now: float, q0: np.ndarray, v0: np.ndarray,
                acc0: np.ndarray, q_final: np.ndarray,
                escala: float = 1.0,
                jerk0: Optional[np.ndarray] = None,
                snap0: Optional[np.ndarray] = None) -> float:
        """Calcula los coeficientes de un nuevo movimiento.

        Args:
            escala: escala de velocidad p (1.0 = 100 %, 0.1 = 10 %).
            jerk0, snap0: 3.ª y 4.ª derivadas iniciales. Si son None y hay
                una trayectoria activa, se toman de ella en t_now; si no,
                se suponen cero.

        Returns:
            Duración T del movimiento [s].
        """
        q0 = np.asarray(q0, dtype=float)
        v0 = np.asarray(v0, dtype=float)
        acc0 = np.asarray(acc0, dtype=float)
        q_final = np.asarray(q_final, dtype=float)

        if jerk0 is None or snap0 is None:
            if self.active and (t_now - self.t0) < self.duration:
                d = self.derivadas(t_now - self.t0, 4)
                j_act, s_act = d[3], d[4]
            else:
                j_act = s_act = np.zeros_like(q0)
            jerk0 = j_act if jerk0 is None else np.asarray(jerk0, float)
            snap0 = s_act if snap0 is None else np.asarray(snap0, float)

        h = q_final - q0
        T = self.duracion(h, escala)

        # Condiciones iniciales en tau: b_k = q^(k)(0) T^k / k!
        b = self.coef
        b[0] = q0
        b[1] = v0 * T
        b[2] = acc0 * T ** 2 / 2.0
        b[3] = jerk0 * T ** 3 / 6.0
        b[4] = snap0 * T ** 4 / 24.0

        # Condiciones finales: s^(m)(1) = qf (m=0), 0 (m=1..4)
        rhs = np.zeros((5, q0.size))
        for m in range(5):
            objetivo = q_final if m == 0 else 0.0
            aporte = np.zeros(q0.size)
            for k in range(m, 5):
                aporte += factorial(k) / factorial(k - m) * b[k]
            rhs[m] = objetivo - aporte
        b[5:] = _A_FIN_INV @ rhs

        self.q_final = q_final.copy()
        self.t0 = t_now
        self.duration = T
        self.active = True
        return T

    # ------------------------------------------------------------------
    def derivadas(self, t: float, n_der: int = 2) -> list:
        """Devuelve [q*, q*', ..., q*^(n_der)] en el tiempo relativo t.

        Evalúa por Horner en tau = t/T y reescala cada derivada por T^-m.
        """
        T = self.duration
        n = self.coef.shape[1]
        if T <= 0.0:
            return [self.q_final.copy()] + [np.zeros(n)] * n_der
        tau = min(max(t / T, 0.0), 1.0)
        out = []
        c = self.coef
        for m in range(n_der + 1):
            acc = np.zeros(n)
            for k in range(ORDEN, m - 1, -1):
                acc = acc * tau + c[k] * (factorial(k) / factorial(k - m))
            out.append(acc / T ** m)
        return out

    def evaluar(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evalúa (q*, q*', q*'') en el tiempo relativo t in [0, T].

        No modifica el estado; sirve para revisar la trayectoria planeada.
        """
        q, qd, qdd = self.derivadas(t, 2)
        return q, qd, qdd

    def muestrear(self, t_now: float
                  ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evalúa (q*, q*', q*'') en el tiempo absoluto t_now.

        Fuera de [0, T] devuelve la posición final con velocidad y
        aceleración nulas y marca la trayectoria como terminada.
        """
        t = t_now - self.t0
        if not self.active or t >= self.duration:
            self.active = False
            zero = np.zeros_like(self.q_final)
            return self.q_final, zero, zero
        return self.evaluar(t)

    def fijar(self, q: np.ndarray) -> None:
        """Fija la referencia en q, sin movimiento."""
        self.q_final = np.asarray(q, dtype=float).copy()
        self.active = False


# Alias para no tocar adrc_simple_node.py (importa TrayectoriaQuintica).
TrayectoriaQuintica = TrayectoriaNonica
