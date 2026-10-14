"""Pruebas del compensador GPI (sin modelo del brazo)."""

import numpy as np
import pytest

from control_adrc_rover.nucleo.gpi import GPI, ganancias_desde_polos


def test_ganancias_son_coeficientes_del_producto():
    k3, k2, k1, k0 = ganancias_desde_polos([1.0, 2.0, 3.0, 4.0])
    # (s+1)(s+2)(s+3)(s+4) = s^4 + 10 s^3 + 35 s^2 + 50 s + 24
    assert (k3, k2, k1, k0) == pytest.approx((10.0, 35.0, 50.0, 24.0))


def _simular(kappa=1.0, phi=0.0, retardo=1, sat=None, n=1200, dt=0.005,
             polos=(8.0, 8.0, 32.0, 32.0), escalon=0.0):
    """Doble integrador q'' = kappa*u + phi regulado a 0 desde q = escalon."""
    g = GPI(np.array([polos]))
    q, qd = np.array([escalon]), np.zeros(1)
    cola = [q.copy()] * (retardo + 1)
    hist = []
    for _ in range(n):
        qm = cola[0]
        v, s2d = g.calcular(qm, np.zeros(1))
        u = v.copy()
        dv = None
        if sat is not None:
            u = np.clip(v, -sat, sat)
            dv = u - v
        g.integrar(s2d, dt, None, dv)
        acc = kappa * u + phi
        qd = qd + acc * dt
        q = q + qd * dt
        cola = cola[1:] + [q.copy()]
        hist.append(q[0])
    return np.array(hist), g


def test_rechaza_perturbacion_constante_y_la_estima():
    hist, g = _simular(phi=2.0)
    assert abs(hist[-1]) < 1e-4
    assert g.phi_hat()[0] == pytest.approx(2.0, rel=1e-3)


@pytest.mark.parametrize('kappa', [0.6, 1.0, 1.4])
def test_sin_sobreimpulso_siguiendo_trayectoria(kappa):
    """Con referencia suave y feedforward, el error nace sólo de kappa y el
    movimiento llega sin pasarse (un ESCALÓN sí se pasaría: por eso el
    control siempre usa trayectorias)."""
    from control_adrc_rover.nucleo.trayectoria import TrayectoriaNonica
    tr = TrayectoriaNonica(np.array([1.0]), np.array([3.0]), 0.5)
    tr.planear(0.0, np.zeros(1), np.zeros(1), np.zeros(1), np.array([0.5]))
    g = GPI(np.array([[8.0, 8.0, 32.0, 32.0]]))
    q, qd, dt = np.zeros(1), np.zeros(1), 0.005
    cola = [q.copy(), q.copy()]
    hist = []
    for k in range(1600):
        qr, _, qddr, _, _ = tr.estado_en(k * dt)
        v, s2d = g.calcular(cola[0] - qr, qddr)
        g.integrar(s2d, dt)
        qd = qd + kappa * v * dt
        q = q + qd * dt
        cola = cola[1:] + [q.copy()]
        hist.append(q[0])
    hist = np.array(hist)
    assert hist.max() - 0.5 < 0.005 * 0.5          # < 0.5 % del recorrido
    assert abs(hist[-1] - 0.5) < 1e-3


def test_inestable_fuera_del_rango_de_kappa():
    hist, _ = _simular(kappa=8.0, escalon=0.01, n=800)
    assert not np.all(np.isfinite(hist)) or np.max(np.abs(hist)) > 1.0


def test_antiwindup_reduce_sobreimpulso_con_saturacion():
    # sin back-calculation: delta_v = None aunque se recorte
    g = GPI(np.array([[8.0, 8.0, 32.0, 32.0]]))
    q, qd = 0.5, 0.0
    sin_aw = []
    for _ in range(1500):
        v, s2d = g.calcular(np.array([q]), np.zeros(1))
        u = float(np.clip(v[0], -3.0, 3.0))
        g.integrar(s2d, 0.005)
        qd += u * 0.005
        q += qd * 0.005
        sin_aw.append(q)
    con_aw, _ = _simular(escalon=0.5, sat=3.0, n=1500)
    assert np.min(con_aw) > np.min(sin_aw)


def test_cambiar_polos_no_salta_el_mando():
    g = GPI(np.array([[8.0, 8.0, 32.0, 32.0]]))
    g.reiniciar(np.array([1.5]))
    antes = g.parte_integral().copy()
    g.fijar_polos(0, [5.0, 5.0, 20.0, 20.0])
    assert g.parte_integral() == pytest.approx(antes)


def test_reiniciar_da_el_mando_pedido():
    g = GPI(np.array([[8.0, 8.0, 32.0, 32.0]]))
    g.reiniciar(np.array([2.5]))
    v, _ = g.calcular(np.zeros(1), np.zeros(1))
    assert v[0] == pytest.approx(2.5)
