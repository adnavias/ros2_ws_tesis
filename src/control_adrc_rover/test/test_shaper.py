"""Pruebas del conformador y de la estimación del modo."""

import math

import numpy as np
import pytest

from control_adrc_rover.nucleo.shaper import (
    ReferenciaConformada, estimar_modo, impulsos)


@pytest.mark.parametrize('tipo', ['zv', 'zvd', 'ei'])
def test_amplitudes_suman_uno(tipo):
    a, t = impulsos(tipo, 2.0, 0.05)
    assert a.sum() == pytest.approx(1.0)
    assert t[0] == 0.0


def _vibracion_residual(tipo, f=2.0, zeta=0.05):
    """Escalón conformado aplicado a un oscilador; amplitud residual."""
    dt = 0.001
    wn = 2 * math.pi * f
    sh = ReferenciaConformada(tipo, f, zeta)
    x = v = 0.0
    hist = []
    for k in range(int(4.0 / dt)):
        t = k * dt
        u, _, _ = sh.aplicar(t, np.array([1.0 if t > 0.1 else 0.0]),
                             np.zeros(1), np.zeros(1))
        acc = wn ** 2 * (u[0] - x) - 2 * zeta * wn * v
        v += acc * dt
        x += v * dt
        hist.append(x)
    cola = np.array(hist[-1500:])
    return cola.max() - cola.min()


def test_zvd_casi_elimina_la_vibracion():
    sin = _vibracion_residual('ninguno')
    con = _vibracion_residual('zvd')
    assert con < 0.05 * sin


def test_estimar_modo_decaimiento():
    fs, f0, z0 = 200.0, 3.0, 0.04
    t = np.arange(0, 6.0, 1 / fs)
    x = np.exp(-z0 * 2 * math.pi * f0 * t) * np.sin(2 * math.pi * f0 * t)
    r = estimar_modo(x + 0.002 * np.random.default_rng(0).standard_normal(
        t.size), fs)
    assert r['frecuencia_hz'] == pytest.approx(f0, rel=0.03)
    assert r['amortiguamiento'] == pytest.approx(z0, rel=0.3)


def test_estimar_modo_ruido_sin_pico():
    x = np.random.default_rng(1).standard_normal(2000)
    assert estimar_modo(x, 200.0) is None
