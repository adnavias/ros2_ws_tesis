"""Pruebas del supervisor, del estimador de kappa y de la identificación."""

import math

import numpy as np
import pytest

from control_adrc_rover.nucleo.identificacion import (
    Muestras, elegir_beta, identificar)
from control_adrc_rover.nucleo.supervisor import (
    EstimadorKappa, Supervisor, analizar)

CFG = {'habilitado': True, 'umbral_sobreimpulso_deg': 0.5,
       'umbral_sobreimpulso_pct': 2.0, 'cruces_max': 2,
       'factor_reduccion': 0.85, 'wn_min': 2.0, 'ventana': 1.5,
       'banda_cruces_deg': 0.05}


def test_analizar_sobreimpulso_y_cruces():
    q = np.radians([10.0, 10.6, 9.6, 10.3, 9.9, 10.0])
    r = analizar('J', q, 0.0, math.radians(10.0), math.radians(0.05))
    assert r.sobreimpulso_deg == pytest.approx(0.6, abs=1e-6)
    assert r.cruces == 3


def test_supervisor_baja_y_nunca_sube():
    s = Supervisor(CFG)
    malo = analizar('J', np.radians([10.0, 11.0, 10.0]), 0.0,
                    math.radians(10.0), math.radians(0.05))
    bueno = analizar('J', np.radians([9.9, 10.0, 10.0]), 0.0,
                     math.radians(10.0), math.radians(0.05))
    assert s.decidir(malo, 10.0, None) == pytest.approx(0.85)
    assert s.decidir(bueno, 10.0, None) is None
    assert s.decidir(malo, 2.0, None) is None          # ya en el mínimo


def test_supervisor_no_baja_si_kappa_es_bajo():
    s = Supervisor(CFG)
    malo = analizar('J', np.radians([10.0, 11.0, 10.0]), 0.0,
                    math.radians(10.0), math.radians(0.05))
    assert s.decidir(malo, 10.0, 0.4) is None
    assert 'autotuning' in s.eventos[-1]


def test_estimador_kappa():
    rng = np.random.default_rng(0)
    est = EstimadorKappa(1, olvido=1.0)
    kappa = 1.6
    for _ in range(2000):
        qdd = rng.uniform(-3, 3)
        phi = (1 - 1 / kappa) * qdd + 0.3 / kappa + 0.01 * rng.standard_normal()
        est.actualizar(np.array([qdd]), np.array([phi]))
    assert est.kappa()[0] == pytest.approx(kappa, rel=0.03)


def test_identificacion_recupera_parametros():
    rng = np.random.default_rng(2)
    m = Muestras()
    alfa, ja, fc, ag = 1.3, 0.6, 1.2, 0.9
    for _ in range(3000):
        mii = rng.choice([0.2, 0.8, 1.6]) + 0.01 * rng.standard_normal()
        qdd = rng.uniform(-3, 3)
        fric = np.tanh(rng.uniform(-3, 3))
        gi = rng.uniform(-10, 10)
        tau = alfa * mii * qdd + ja * qdd + fc * fric + ag * gi
        m.agregar(tau + 0.02 * rng.standard_normal(), qdd, fric, mii, gi)
    r = identificar(m, armadura_nominal=0.8)
    assert r.alfa == pytest.approx(alfa, rel=0.02)
    assert r.armadura == pytest.approx(ja, rel=0.05)
    assert r.friccion_tau == pytest.approx(fc, rel=0.05)
    assert r.alfa_gravedad == pytest.approx(ag, rel=0.02)
    assert not r.armadura_fijada


def test_beta_cubre_incertidumbre():
    assert elegir_beta(1.0, 0.0) == pytest.approx(1.0)
    assert elegir_beta(1.0, 0.1) == pytest.approx(1 / 1.2)
    assert elegir_beta(1.0, 2.0) == pytest.approx(0.6)
