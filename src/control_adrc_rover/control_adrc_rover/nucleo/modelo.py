"""Modelo dinámico del brazo (Pinocchio) con articulaciones activas,
bloqueadas y ausentes.

A partir del URDF completo se construye un modelo REDUCIDO cuyas
coordenadas libres son sólo las articulaciones activas con mando
"corriente_con_modelo":

    ausente    -> se anula la inercia de todo su subárbol (no está montado)
                  y la articulación se bloquea (su posición ya no importa).
    bloqueada  -> se bloquea en `posicion_bloqueo` con pinocchio.
                  buildReducedModel; su masa sigue contando.
    activa sin modelo (torque_sin_modelo, posicion_directa)
               -> se bloquea en `posicion_bloqueo` (su masa cuenta, su
                  acoplamiento dinámico lo absorbe el observador).

Para la configuración q de las coordenadas libres devuelve

    M(q)  matriz de inercia de los eslabones (CRBA), sin rotores,
    g(q)  pares de gravedad.

La calibración (autotune o edición a mano) se aplica encima:

    M_cal(q) = D M(q) D + diag(J_a),   D = diag(sqrt(alfa))
    g_cal(q) = alfa_g ⊙ g(q)

y el control usa la cota baja  M_ctrl = S M_cal S,  S = diag(sqrt(beta)),
mientras que la verificación de trayectorias usa la cota alta
M_sup = f_sup M_cal  y  g_sup = f_g g_cal (ver controlador.py).

Las articulaciones `continuous` usan dos coordenadas (cos q, sin q) en
Pinocchio; la conversión es interna.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .config import ConfigBrazo, ErrorConfig, validar_arbol


class ModeloBrazo:
    """M(q) y g(q) del modelo reducido a las articulaciones con modelo."""

    def __init__(self, urdf_path: str, cfg: ConfigBrazo) -> None:
        """Carga el URDF, anula los subárboles ausentes y reduce el modelo.

        También completa en `cfg` los límites articulares del URDF y
        valida que lo que cuelga de una articulación ausente esté ausente.
        """
        import pinocchio as pin
        self._pin = pin
        completo = pin.buildModelFromUrdf(urdf_path)

        nombres_perfil = cfg.nombres
        for nom in nombres_perfil:
            if not completo.existJointName(nom):
                urdf_joints = [completo.names[i]
                               for i in range(1, completo.njoints)]
                raise ErrorConfig(
                    f'El URDF {urdf_path} no tiene la articulación "{nom}". '
                    f'Articulaciones del URDF: {urdf_joints}')

        # Límites del URDF y árbol de padres entre articulaciones del perfil.
        padres: Dict[str, Optional[str]] = {}
        for a in cfg.articulaciones:
            jid = completo.getJointId(a.nombre)
            joint = completo.joints[jid]
            a.continua = joint.nq == 2
            if not a.continua:
                lo = float(completo.lowerPositionLimit[joint.idx_q])
                hi = float(completo.upperPositionLimit[joint.idx_q])
                if hi > lo and math.isfinite(lo) and math.isfinite(hi):
                    a.limite_inferior, a.limite_superior = lo, hi
            p = completo.parents[jid]
            padre = None
            while p > 0:
                nom_p = completo.names[p]
                if nom_p in nombres_perfil:
                    padre = nom_p
                    break
                p = completo.parents[p]
            padres[a.nombre] = padre
        validar_arbol(cfg, padres)
        self.padres = padres

        # Copia con inercias nulas en los subárboles ausentes.
        modelo = pin.buildModelFromUrdf(urdf_path)  # copia independiente
        for a in cfg.por_estado('ausente'):
            jid = modelo.getJointId(a.nombre)
            for k in modelo.subtrees[jid]:
                modelo.inertias[k] = pin.Inertia.Zero()

        # Articulaciones a bloquear y su posición de bloqueo.
        q_ref = pin.neutral(modelo)
        bloquear: List[int] = []
        self.extra_urdf: List[str] = []
        for jid in range(1, modelo.njoints):
            nom = modelo.names[jid]
            joint = modelo.joints[jid]
            if joint.nq == 0:
                continue
            if nom in nombres_perfil:
                a = cfg.articulacion(nom)
                if a.en_modelo:
                    continue
                pos = a.posicion_bloqueo if a.estado != 'ausente' else 0.0
            else:
                self.extra_urdf.append(nom)
                pos = 0.0
            if joint.nq == 2:
                q_ref[joint.idx_q] = math.cos(pos)
                q_ref[joint.idx_q + 1] = math.sin(pos)
            elif joint.nq == 1:
                q_ref[joint.idx_q] = pos
            bloquear.append(jid)

        self.reducido = (pin.buildReducedModel(modelo, bloquear, q_ref)
                         if bloquear else modelo)
        self.nombres = [a.nombre for a in cfg.articulaciones if a.en_modelo]
        self.n = len(self.nombres)
        self._idx_q = np.zeros(self.n, dtype=int)
        self._idx_v = np.zeros(self.n, dtype=int)
        self._cont = np.zeros(self.n, dtype=bool)
        for k, nom in enumerate(self.nombres):
            joint = self.reducido.joints[self.reducido.getJointId(nom)]
            self._idx_q[k] = joint.idx_q
            self._idx_v[k] = joint.idx_v
            self._cont[k] = joint.nq == 2
        if self.reducido.nv != self.n:
            raise ErrorConfig(
                f'Modelo reducido con {self.reducido.nv} GDL y se esperaban '
                f'{self.n} ({self.nombres}); revisa el URDF.')
        self._sel = np.ix_(self._idx_v, self._idx_v)
        self._q0 = pin.neutral(self.reducido)
        self.data = self.reducido.createData()

    # ------------------------------------------------------------------
    def nuevo_data(self):
        """Data de Pinocchio independiente (uno por hilo)."""
        return self.reducido.createData()

    def _config(self, q: np.ndarray) -> np.ndarray:
        qp = self._q0.copy()
        for k in range(self.n):
            i = self._idx_q[k]
            if self._cont[k]:
                qp[i] = math.cos(q[k])
                qp[i + 1] = math.sin(q[k])
            else:
                qp[i] = q[k]
        return qp

    def calcular(self, q: np.ndarray, data=None
                 ) -> Tuple[np.ndarray, np.ndarray]:
        """(M(q), g(q)) de los eslabones, en el orden de `self.nombres`."""
        if self.n == 0:
            return np.zeros((0, 0)), np.zeros(0)
        pin = self._pin
        data = self.data if data is None else data
        qp = self._config(np.asarray(q, dtype=float))
        m_up = pin.crba(self.reducido, data, qp)
        m = np.triu(m_up) + np.triu(m_up, 1).T
        g = pin.computeGeneralizedGravity(self.reducido, data, qp)
        return m[self._sel].copy(), np.asarray(g)[self._idx_v].copy()


def calibrar(m: np.ndarray, g: np.ndarray, alfa: np.ndarray,
             armadura: np.ndarray, alfa_g: np.ndarray
             ) -> Tuple[np.ndarray, np.ndarray]:
    """Aplica la calibración: D M D + diag(J_a) y alfa_g ⊙ g."""
    d = np.sqrt(alfa)
    return (d[:, None] * m * d[None, :] + np.diag(armadura),
            alfa_g * g)


def escalar_beta(m: np.ndarray, beta: np.ndarray) -> np.ndarray:
    """Cota baja para el control: S M S con S = diag(sqrt(beta))."""
    s = np.sqrt(beta)
    return s[:, None] * m * s[None, :]
