"""Planta simple del brazo en Python (sin Gazebo) para pruebas.

Integra la dinámica rígida  (M(q) + diag(J_a)) q'' + C q' + g(q) = tau
con Pinocchio (algoritmo ABA) a 1 kHz, con:

    - masas e inercias de cada eslabón escaladas por un factor,
    - articulaciones ausentes (inercia nula) y bloqueadas,
    - armadura y K_tau "reales" distintos de los del controlador,
    - zona muerta de corriente i0 (igual que actuador_sim) o fricción de
      Coulomb que se opone a la velocidad (más parecida al motor real),
    - fricción viscosa pequeña,
    - medición retrasada un número entero de muestras del control.

Se usa en las pruebas de pytest (Monte Carlo) y como banco de pruebas
rápido fuera de ROS.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import math
from collections import deque
from typing import Dict, List, Optional

import numpy as np


class Planta:
    """Brazo simulado con parámetros "reales" configurables."""

    def __init__(self, urdf: str, nombres: List[str],
                 estados: Dict[str, str], q0: Dict[str, float],
                 escala_masa: Optional[Dict[str, float]] = None,
                 armadura: Optional[np.ndarray] = None,
                 k_tau: Optional[np.ndarray] = None,
                 i0: Optional[np.ndarray] = None,
                 viscosa=0.05,
                 retardo_muestras: int = 1,
                 coulomb: Optional[np.ndarray] = None) -> None:
        """Args:
            nombres: articulaciones que la planta mueve (en este orden se
                reciben las corrientes y se entregan las posiciones).
            estados: estado de cada articulación del URDF que no esté en
                `nombres`: 'ausente' (sin masa) o 'bloqueada'.
            q0: postura inicial/bloqueo de cada articulación [rad URDF].
            coulomb: fricción de Coulomb real [N·m] (se opone a la
                velocidad), alternativa a la zona muerta i0.
            escala_masa: factor de masa e inercia por eslabón hijo de cada
                articulación (p. ej. {'Joint_2': 1.3}).
        """
        import pinocchio as pin
        self._pin = pin
        modelo = pin.buildModelFromUrdf(urdf)
        escala_masa = escala_masa or {}
        for nom, factor in escala_masa.items():
            jid = modelo.getJointId(nom)
            ine = modelo.inertias[jid]
            modelo.inertias[jid] = pin.Inertia(
                ine.mass * factor, ine.lever, ine.inertia * factor)
        for nom, est in estados.items():
            if est == 'ausente':
                jid = modelo.getJointId(nom)
                for k in modelo.subtrees[jid]:
                    modelo.inertias[k] = pin.Inertia.Zero()
        q_ref = pin.neutral(modelo)
        bloquear = []
        for jid in range(1, modelo.njoints):
            nom = modelo.names[jid]
            j = modelo.joints[jid]
            if nom in nombres or j.nq == 0:
                continue
            pos = q0.get(nom, 0.0)
            if j.nq == 2:
                q_ref[j.idx_q], q_ref[j.idx_q + 1] = math.cos(pos), math.sin(pos)
            else:
                q_ref[j.idx_q] = pos
            bloquear.append(jid)
        self.modelo = (pin.buildReducedModel(modelo, bloquear, q_ref)
                       if bloquear else modelo)
        self.data = self.modelo.createData()
        self.n = len(nombres)
        self.nombres = list(nombres)
        self.idx_q, self.idx_v, self.cont = [], [], []
        for nom in nombres:
            j = self.modelo.joints[self.modelo.getJointId(nom)]
            self.idx_q.append(j.idx_q)
            self.idx_v.append(j.idx_v)
            self.cont.append(j.nq == 2)
        self.armadura = (np.zeros(self.n) if armadura is None
                         else np.asarray(armadura, dtype=float))
        self.k_tau = (np.ones(self.n) if k_tau is None
                      else np.asarray(k_tau, dtype=float))
        self.i0 = np.zeros(self.n) if i0 is None else np.asarray(i0, float)
        self.viscosa = np.broadcast_to(np.asarray(viscosa, dtype=float),
                                       (self.n,)).copy()
        self.coulomb = (np.zeros(self.n) if coulomb is None
                        else np.asarray(coulomb, dtype=float))
        self.q = np.array([q0.get(n, 0.0) for n in nombres], dtype=float)
        self.qd = np.zeros(self.n)
        self.retardo = int(retardo_muestras)
        self._cola: deque = deque()

    def _config(self, q):
        qp = self._pin.neutral(self.modelo)
        for k in range(self.n):
            i = self.idx_q[k]
            if self.cont[k]:
                qp[i], qp[i + 1] = math.cos(q[k]), math.sin(q[k])
            else:
                qp[i] = q[k]
        return qp

    def torque(self, corriente: np.ndarray) -> np.ndarray:
        """Corriente -> torque con zona muerta i0 (como actuador_sim)."""
        util = np.maximum(np.abs(corriente) - self.i0, 0.0)
        return self.k_tau * np.sign(corriente) * util

    def avanzar(self, corriente: np.ndarray, dt: float,
                sub: int = 5) -> None:
        """Integra dt con `sub` subpasos (semi-implícito)."""
        self.avanzar_torque(self.torque(np.asarray(corriente, dtype=float)),
                            dt, sub)

    def avanzar_torque(self, tau_m: np.ndarray, dt: float,
                       sub: int = 5) -> None:
        """Integra dt con el torque articular dado (sin modelo de motor)."""
        pin = self._pin
        tau_m = np.asarray(tau_m, dtype=float)
        h = dt / sub
        for _ in range(sub):
            qp = self._config(self.q)
            v = np.zeros(self.modelo.nv)
            v[self.idx_v] = self.qd
            m = pin.crba(self.modelo, self.data, qp)
            m = np.triu(m) + np.triu(m, 1).T
            nle = pin.nonLinearEffects(self.modelo, self.data, qp, v)
            sel = np.ix_(self.idx_v, self.idx_v)
            m_r = m[sel] + np.diag(self.armadura)
            rhs = (tau_m - nle[self.idx_v] - self.viscosa * self.qd
                   - self.coulomb * np.tanh(self.qd / 0.005))
            qdd = np.linalg.solve(m_r, rhs)
            self.qd = self.qd + qdd * h
            self.q = self.q + self.qd * h

    def medir(self) -> np.ndarray:
        """Posición medida (con el retardo configurado)."""
        self._cola.append(self.q.copy())
        while len(self._cola) > self.retardo + 1:
            self._cola.popleft()
        return self._cola[0].copy()


def simular(ctrl, planta: Planta, t_total: float,
            objetivos: Optional[List[tuple]] = None,
            idx_planta: Optional[np.ndarray] = None) -> dict:
    """Corre controlador + planta.

    Args:
        objetivos: lista de (t, vector_usuario_rad, pct) a mandar.
        idx_planta: índices (en ctrl.movibles) de lo que mueve la planta,
            en el orden de planta.nombres. Por defecto, los GPI.
    Returns:
        dict con t, q (usuario), q_d, cmd y estados.
    """
    dt = ctrl.dt_nominal
    if idx_planta is None:
        idx_planta = ctrl.k_gpi
    objetivos = sorted(objetivos or [], key=lambda x: x[0])
    reg = {'t': [], 'q': [], 'q_d': [], 'cmd': [], 'estado': []}
    t = 0.0
    pendientes = list(objetivos)
    lista_planta = list(idx_planta)
    while t < t_total:
        q_med = planta.medir()
        raw = np.zeros(len(ctrl.k_med))
        # orden de k_med; las medidas que la planta no simula quedan en 0
        for j, k in enumerate(ctrl.k_med):
            if k in lista_planta:
                raw[j] = (q_med[lista_planta.index(k)]
                          - ctrl.offset[k]) * ctrl.signo[k]
        if ctrl.medir(raw, t):
            mando = ctrl.paso(t)
        else:
            mando = None
        while pendientes and pendientes[0][0] <= t:
            _, obj, pct = pendientes.pop(0)
            ctrl.mover(np.asarray(obj, dtype=float), pct)
        corr = np.zeros(planta.n)
        if mando is not None:
            corr = mando.valores[idx_planta] * ctrl.signo[idx_planta]
        planta.avanzar(corr, dt)
        reg['t'].append(t)
        reg['q'].append(ctrl.a_usuario(ctrl.q).copy())
        reg['q_d'].append(ctrl.a_usuario(ctrl.q_d).copy())
        reg['cmd'].append(None if mando is None else mando.valores.copy())
        reg['estado'].append(ctrl.estado)
        t += dt
    for clave in ('t', 'q', 'q_d'):
        reg[clave] = np.array(reg[clave])
    return reg
