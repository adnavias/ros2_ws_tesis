"""Pruebas de las protecciones."""

import math

import numpy as np
import pytest

from control_adrc_rover.nucleo import seguridad as S


def test_nan_y_salto_se_rechazan_y_falla_tras_varias():
    s = S.SanidadEncoder(np.array([0.5]), np.array([False]), invalidas_max=3)
    q, ok, falla = s.procesar(np.array([1.0]))
    assert ok[0] and q[0] == 1.0
    q, ok, falla = s.procesar(np.array([np.nan]))
    assert not ok[0] and q[0] == 1.0
    q, ok, falla = s.procesar(np.array([3.0]))          # salto de 2 rad
    assert not ok[0] and q[0] == 1.0 and not falla
    for _ in range(3):
        _, _, falla = s.procesar(np.array([np.nan]))
    assert falla


def test_desenvuelve_vueltas_de_articulacion_continua():
    s = S.SanidadEncoder(np.array([0.5]), np.array([True]))
    s.procesar(np.array([math.pi - 0.05]))
    q, ok, _ = s.procesar(np.array([-math.pi + 0.05]))
    assert ok[0]
    assert q[0] == pytest.approx(math.pi + 0.05)


def test_watchdog():
    assert S.estado_watchdog(0.01, 0.02, 0.1) == 'ok'
    assert S.estado_watchdog(0.05, 0.02, 0.1) == 'congelar'
    assert S.estado_watchdog(0.2, 0.02, 0.1) == 'quitar'


def test_pared_virtual():
    a = S.aceleracion_pared(np.array([0.0, 2.98, -2.98]), np.zeros(3),
                            np.full(3, -3.0), np.full(3, 3.0),
                            np.full(3, 0.05), np.full(3, 400.0),
                            np.full(3, 40.0))
    assert a[0] == 0.0
    assert a[1] < 0.0 and a[2] > 0.0


def test_limitador_de_pendiente():
    lim = S.LimitadorPendiente(np.array([100.0]))
    lim.aplicar(np.array([0.0]), 0.005)
    out = lim.aplicar(np.array([10.0]), 0.005)
    assert out[0] == pytest.approx(0.5)


def test_objetivo_fuera_de_rango():
    obj, avisos = S.validar_objetivo(np.array([4.0]), np.array([-3.0]),
                                     np.array([3.0]), np.array([0.05]),
                                     'recortar', ['J'])
    assert obj[0] == pytest.approx(2.95) and avisos
    with pytest.raises(ValueError):
        S.validar_objetivo(np.array([4.0]), np.array([-3.0]),
                           np.array([3.0]), np.array([0.05]), 'rechazar',
                           ['J'])
