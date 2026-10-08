"""Modelo dinámico del brazo a partir del URDF (Pinocchio).

Calcula, para la configuración articular q:

    M(q)  matriz de inercia 5x5 (algoritmo CRBA) + inercia reflejada de
          los rotores en la diagonal (armadura J_rotor * N^2),
    g(q)  vector de pares de gravedad.

Es la misma descripción (brazo.urdf) que usa Gazebo, de modo que el modelo
del controlador y el de la simulación son coherentes. La base del brazo se
supone fija y nivelada (el movimiento del chasis lo absorbe el ADRC como
perturbación); Coriolis y fricción también los absorbe el observador.

Las articulaciones `continuous` en Pinocchio usan dos coordenadas
(cos q, sin q); esta clase hace la conversión internamente.

Autor: Adán Medina Covarrubias
"""

from typing import List, Tuple

import numpy as np
import pinocchio as pin


class ModeloBrazo:
    """M(q) y g(q) del brazo para las articulaciones dadas."""

    def __init__(self, urdf_path: str, joint_names: List[str],
                 armadura: np.ndarray) -> None:
        """Carga el URDF y cachea los índices de cada articulación.

        Args:
            urdf_path: ruta al URDF del brazo (brazo.urdf).
            joint_names: nombres en el orden del controlador.
            armadura: inercia reflejada de cada rotor [kg m^2].
        """
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.n = len(joint_names)
        self.idx_q = np.zeros(self.n, dtype=int)
        self.idx_v = np.zeros(self.n, dtype=int)
        self.is_continuous = np.zeros(self.n, dtype=bool)
        for k, name in enumerate(joint_names):
            if not self.model.existJointName(name):
                raise ValueError(f'El URDF no tiene la articulación {name}')
            joint = self.model.joints[self.model.getJointId(name)]
            self.idx_q[k] = joint.idx_q
            self.idx_v[k] = joint.idx_v
            self.is_continuous[k] = joint.nq == 2
        if self.model.nv != self.n:
            raise ValueError(
                f'El URDF tiene {self.model.nv} GDL y se esperaban {self.n}')
        self.armadura = np.diag(np.asarray(armadura, dtype=float))
        self._q_pin = pin.neutral(self.model)
        self._sel = np.ix_(self.idx_v, self.idx_v)

    def _config(self, q: np.ndarray) -> np.ndarray:
        """Convierte ángulos [rad] al vector de configuración de Pinocchio."""
        qp = self._q_pin
        for k in range(self.n):
            i = self.idx_q[k]
            if self.is_continuous[k]:
                qp[i] = np.cos(q[k])
                qp[i + 1] = np.sin(q[k])
            else:
                qp[i] = q[k]
        return qp

    def calcular(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Devuelve (M(q) + armadura, g(q)) en el orden del controlador."""
        qp = self._config(q)
        m_up = pin.crba(self.model, self.data, qp)
        m_full = np.triu(m_up) + np.triu(m_up, 1).T   # CRBA llena la mitad
        g = pin.computeGeneralizedGravity(self.model, self.data, qp)
        return m_full[self._sel] + self.armadura, np.asarray(g)[self.idx_v]
