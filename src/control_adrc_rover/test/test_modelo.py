"""Pruebas del modelo dinámico con articulaciones activas/bloqueadas/ausentes."""

import math

import numpy as np
import pytest

from conftest import requiere_pinocchio
from control_adrc_rover.nucleo.config import ErrorConfig

pytestmark = requiere_pinocchio


def _modelo(cfg):
    from control_adrc_rover.nucleo.modelo import ModeloBrazo
    return ModeloBrazo(cfg.urdf, cfg)


def test_modelo_completo_y_limites(cargar):
    cfg = cargar('sim')
    m = _modelo(cfg)
    assert m.nombres == ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4']
    j3 = cfg.articulacion('Joint_3')
    assert j3.limite_inferior == pytest.approx(-3.1416)
    assert cfg.articulacion('Joint_1').continua
    M, g = m.calcular(np.array([math.pi, -math.pi / 2, 0.0, 0.0]))
    assert M.shape == (4, 4)
    assert np.allclose(M, M.T)
    assert np.all(np.linalg.eigvalsh(M) > 0)
    assert g[1] > 15.0          # hombro horizontal: gravedad grande


def test_ausentes_quitan_masa(cargar):
    q = np.array([math.pi, -math.pi / 2])
    completo = _modelo(cargar('sim', estados={'Joint_3': 'bloqueada',
                                              'Joint_4': 'bloqueada',
                                              'Joint_5': 'bloqueada'}))
    ausente = _modelo(cargar('sim', montadas=2))
    Mc, gc = completo.calcular(q)
    Ma, ga = ausente.calcular(q)
    # con J3..J5 montadas M22 es ~28 veces la de J1+J2 solas
    assert Mc[1, 1] / Ma[1, 1] > 20.0
    assert abs(gc[1]) > abs(ga[1])


def test_solo_j1(cargar):
    cfg = cargar('real', montadas=1)
    m = _modelo(cfg)
    assert m.nombres == ['Joint_1']
    M, g = m.calcular(np.array([0.3]))
    assert M.shape == (1, 1) and M[0, 0] > 0.0
    assert abs(g[0]) < 1e-9          # eje vertical: sin gravedad


def test_hijo_activo_de_ausente_se_rechaza(cargar):
    cfg = cargar('sim', estados={'Joint_2': 'ausente'})
    with pytest.raises(ErrorConfig, match='cuelga de Joint_2'):
        _modelo(cfg)
