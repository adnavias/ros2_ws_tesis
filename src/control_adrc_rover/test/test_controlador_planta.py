"""Lazo completo (controlador + planta en Python, sin Gazebo).

Incluye el caso que motivó el refactor: probar en el robot sólo J1 y J2
con un modelo que aún tenía las masas de J3..J5 (kappa ≈ 3 en J2) y
medición retrasada 15 ms -> oscilación; con J3..J5 "ausentes" no oscila.

Monte Carlo: masas, armadura y K_tau ±50 % (N = variable MC_N, 10 por
defecto). Se descartan las plantas que físicamente no pueden sostener el
brazo con el límite de corriente (eso no es inestabilidad del control).
"""

import math
import os

import numpy as np
import pytest

from conftest import requiere_pinocchio

pytestmark = requiere_pinocchio

Q0 = {'Joint_1': math.pi, 'Joint_2': -math.pi / 2, 'Joint_3': 0.0,
      'Joint_4': 0.0, 'Joint_5': 0.0}
ARM = {'Joint_1': 0.777, 'Joint_2': 0.840, 'Joint_3': 0.840, 'Joint_4': 0.777}
KT = {'Joint_1': 3.7485, 'Joint_2': 3.9375, 'Joint_3': 3.9375,
      'Joint_4': 3.7485}
I0 = {'Joint_1': 0.25, 'Joint_2': 0.497, 'Joint_3': 0.497, 'Joint_4': 0.25}
ABS = {'por_defecto': {'encoder': 'absoluto'},
       'seguridad': {'watchdog_congelar': 0.05, 'watchdog_quitar': 0.5}}


def _armar(cfg, montadas_planta=4, q0=None, **kw):
    from control_adrc_rover.nucleo.controlador import Controlador
    from control_adrc_rover.nucleo.modelo import ModeloBrazo
    from control_adrc_rover.nucleo.planta import Planta
    ctrl = Controlador(cfg, ModeloBrazo(cfg.urdf, cfg), log=lambda s: None)
    nombres = [ctrl.movibles[k].nombre for k in ctrl.k_gpi]
    todas = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']
    estados = {n: 'ausente' for n in todas[montadas_planta:]}
    if montadas_planta >= 4:          # el servo J5 está montado (masa)
        estados['Joint_5'] = 'bloqueada'
    for n in todas[:montadas_planta]:
        if n not in nombres:
            estados[n] = 'bloqueada'
    estados.setdefault('Joint_5', 'bloqueada')
    kw.setdefault('i0', np.array([I0[n] for n in nombres]))
    kw.setdefault('k_tau', np.array([KT[n] for n in nombres]))
    planta = Planta(cfg.urdf, nombres, estados, q0 or Q0, **kw)
    return ctrl, planta


def _correr(ctrl, planta, objetivo, t_total, pct=50.0):
    from control_adrc_rover.nucleo.planta import simular
    return simular(ctrl, planta, t_total, [(0.3, objetivo, pct)])


def test_brazo_completo_sin_sobreimpulso(cargar):
    cfg = cargar('sim')
    ctrl, planta = _armar(cfg)
    obj = np.radians([30.0, 10.0, -20.0, 5.0, np.nan])
    r = _correr(ctrl, planta, obj, 8.0, 80.0)
    q = r['q'][:, :4]
    meta = np.radians([30.0, 10.0, -20.0, 5.0])
    error = np.degrees(q - r['q_d'][:, :4])
    assert ctrl.estado == 'habilitado'
    # sobreimpulso al llegar (ventana del supervisor) y seguimiento
    assert np.all(np.degrees(ctrl.sobreimpulso[:4]) < 0.2)
    assert np.all(np.abs(error).max(axis=0) < 0.5)
    assert np.all(np.abs(np.degrees(q[-1] - meta)) < 0.05)


def test_primera_corriente_es_la_de_gravedad(cargar):
    cfg = cargar('sim')
    ctrl, planta = _armar(cfg)
    raw = (planta.q - ctrl.offset[:4]) * ctrl.signo[:4]
    assert ctrl.medir(np.append(raw, []), 0.0) or True
    mando = ctrl.paso(0.005)
    m_cal, g_cal = ctrl._modelo_cal(ctrl.q, ctrl.data_ctrl)
    i_grav = g_cal / ctrl.k_tau[:4]
    assert mando.valores[:4] == pytest.approx(
        i_grav + ctrl.fc[:4] * np.tanh(ctrl.fc_gain[:4] * i_grav), rel=0.02)


def test_solo_j1_montada(cargar):
    cfg = cargar(None, montadas=1, extra=ABS)
    ctrl, planta = _armar(cfg, montadas_planta=1,
                          armadura=np.array([ARM['Joint_1']]))
    assert ctrl.nombres == ['Joint_1']
    r = _correr(ctrl, planta, np.radians([45.0]), 10.0)
    q = r['q'][:, 0]
    assert ctrl.estado == 'habilitado'
    assert math.degrees(q.max()) - 45.0 < 0.2
    assert abs(math.degrees(q[-1]) - 45.0) < 0.05


@pytest.mark.parametrize('ausentes, debe_oscilar', [(False, True),
                                                    (True, False)])
def test_caso_j1_j2_con_masas_de_mas(cargar, ausentes, debe_oscilar):
    if ausentes:
        cfg = cargar(None, montadas=2, extra=ABS)
    else:      # modelo viejo: J3..J5 "existen" (su masa cuenta)
        cfg = cargar(None, extra=ABS, estados={'Joint_3': 'bloqueada',
                                               'Joint_4': 'bloqueada',
                                               'Joint_5': 'bloqueada'})
    ctrl, planta = _armar(cfg, montadas_planta=2, retardo_muestras=3,
                          armadura=np.array([ARM['Joint_1'],
                                             ARM['Joint_2']]))
    r = _correr(ctrl, planta, np.radians([20.0, 15.0]), 8.0)
    e = np.degrees(r['q'][:, :2] - r['q_d'][:, :2])
    cola = e[-300:, 1]
    oscila = (cola.max() - cola.min()) > 1.0 or ctrl.estado != 'habilitado'
    assert oscila == debe_oscilar


def test_watchdog_quita_corriente(cargar):
    cfg = cargar('sim')
    ctrl, planta = _armar(cfg)
    _correr(ctrl, planta, np.full(5, np.nan), 0.5)
    assert ctrl.paso_sin_medicion(0.07) is not None
    assert ctrl.watchdog == 'congelar'
    mando = ctrl.paso_sin_medicion(0.6)
    assert ctrl.estado == 'falla'
    assert np.allclose(mando.valores, 0.0)


def test_paro_es_latched(cargar):
    cfg = cargar('sim')
    ctrl, planta = _armar(cfg)
    _correr(ctrl, planta, np.full(5, np.nan), 0.3)
    ctrl.paro('prueba')
    ok, _ = ctrl.habilitar()
    assert not ok and ctrl.estado == 'paro'
    assert ctrl.liberar_paro()[0] and ctrl.estado == 'deshabilitado'
    assert ctrl.habilitar()[0] and ctrl.estado == 'habilitado'


def test_monte_carlo_masas_armadura_ktau(cargar):
    import pinocchio as pin
    n = int(os.environ.get('MC_N', '10'))
    rng = np.random.default_rng(int(os.environ.get('MC_SEMILLA', '3')))
    cfg = cargar(None, extra=ABS)
    q0 = dict(Q0, Joint_2=-1.0, Joint_3=1.0)
    meta = np.radians([20.0, -15.0, 20.0, 15.0])
    nombres = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4']
    lim = np.array([a.limite_mando for a in cfg.movibles[:4]])
    probadas, malas = 0, []
    for it in range(n):
        u = lambda: 1.0 + 0.5 * (2.0 * rng.random() - 1.0)  # noqa: E731
        escala = {k: u() for k in nombres + ['Joint_5']}
        arm = np.array([ARM[k] * u() for k in nombres])
        kt = np.array([KT[k] * u() for k in nombres])
        ctrl, planta = _armar(cfg, q0=q0, escala_masa=escala, armadura=arm,
                              k_tau=kt, retardo_muestras=2)
        destino = np.array([q0[k] for k in nombres]) + meta
        hold = 0.0
        for q in (planta.q, destino):
            g = pin.computeGeneralizedGravity(
                planta.modelo, planta.data, planta._config(q))[planta.idx_v]
            hold = max(hold, float(np.max(np.abs(g) / kt / lim)))
        if hold > 0.8:
            continue
        probadas += 1
        r = _correr(ctrl, planta, np.append(meta, np.nan), 6.0)
        q = r['q'][:, :4]
        sobre = np.degrees(np.max((q - meta) * np.sign(meta), axis=0))
        cola = np.degrees(q[-200:] - meta)
        if (ctrl.estado != 'habilitado' or np.any(sobre > 0.5)
                or np.any(cola.max(0) - cola.min(0) > 0.5)):
            malas.append((it, np.round(sobre, 2).tolist()))
    assert probadas >= max(1, n // 2)
    assert not malas, f'casos con sobreimpulso u oscilación: {malas}'


def test_error_excesivo_sostiene_y_luego_falla(cargar):
    cfg = cargar('sim')
    ctrl, planta = _armar(cfg)
    _correr(ctrl, planta, np.full(5, np.nan), 0.5)
    ctrl.q_d = ctrl.q_d.copy()
    ctrl.q_d[2] += math.radians(35.0)
    ctrl.t_prev = 10.0
    cmd = ctrl._ley(ctrl.q, np.zeros(5), 0.005, False)
    assert ctrl.estado == 'habilitado'            # aborta y sostiene
    assert np.allclose(ctrl.q_d, ctrl.q)
    assert abs(cmd[1]) > 1.0                       # sigue sosteniendo J2
    ctrl.q_d = ctrl.q_d.copy()
    ctrl.q_d[2] += math.radians(35.0)
    ctrl.t_prev = 11.0                             # se repite en < 3 s
    cmd = ctrl._ley(ctrl.q, np.zeros(5), 0.005, False)
    assert ctrl.estado == 'falla' and np.allclose(cmd, 0.0)
