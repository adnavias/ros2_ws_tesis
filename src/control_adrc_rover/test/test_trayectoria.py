"""Pruebas de la trayectoria de 9.º grado."""

import numpy as np
import pytest

from control_adrc_rover.nucleo.trayectoria import TrayectoriaNonica


def _tr(j_max=None):
    return TrayectoriaNonica(np.array([1.0, 2.0]), np.array([2.0, 4.0]), 0.5,
                             j_max)


def test_condiciones_de_frontera():
    tr = _tr()
    T = tr.planear(0.0, np.zeros(2), np.zeros(2), np.zeros(2),
                   np.array([1.0, -0.5]))
    d0 = tr.derivadas(0.0, 4)
    dT = tr.derivadas(T, 4)
    assert d0[0] == pytest.approx([0.0, 0.0])
    assert dT[0] == pytest.approx([1.0, -0.5])
    for m in range(1, 5):
        assert np.allclose(d0[m], 0.0, atol=1e-9)
        assert np.allclose(dT[m], 0.0, atol=1e-6)


def test_respeta_velocidad_y_aceleracion_y_es_monotona():
    tr = _tr()
    T = tr.planear(0.0, np.zeros(2), np.zeros(2), np.zeros(2),
                   np.array([1.0, 1.0]))
    ts = np.linspace(0, T, 2000)
    q = np.array([tr.derivadas(t, 2)[0] for t in ts])
    qd = np.array([tr.derivadas(t, 2)[1] for t in ts])
    qdd = np.array([tr.derivadas(t, 2)[2] for t in ts])
    assert np.all(np.diff(q[:, 0]) >= -1e-12)
    assert np.max(np.abs(qd[:, 0])) <= 1.0 + 1e-6
    assert np.max(np.abs(qdd[:, 0])) <= 2.0 + 1e-6


def test_limite_de_jerk_alarga():
    sin = _tr().planear(0, np.zeros(2), np.zeros(2), np.zeros(2),
                        np.array([1.0, 0.0]))
    con = _tr(np.array([5.0, 0.0])).planear(
        0, np.zeros(2), np.zeros(2), np.zeros(2), np.array([1.0, 0.0]))
    assert con > sin
    assert con == pytest.approx(np.cbrt(78.75 * 1.0 / 5.0), rel=1e-6)


def test_replaneo_continuo():
    tr = _tr()
    tr.planear(0.0, np.zeros(2), np.zeros(2), np.zeros(2),
               np.array([1.0, 1.0]))
    antes = tr.estado_en(0.4)
    tr.planear(0.4, antes[0], antes[1], antes[2], np.array([-1.0, 0.5]))
    despues = tr.estado_en(0.4)
    for m in range(5):
        assert despues[m] == pytest.approx(antes[m], abs=1e-6)


def test_duracion_minima_extra():
    tr = _tr()
    T = tr.planear(0.0, np.zeros(2), np.zeros(2), np.zeros(2),
                   np.array([0.01, 0.0]), duracion_min=3.0)
    assert T == pytest.approx(3.0)
