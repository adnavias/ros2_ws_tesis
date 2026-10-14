"""Autotuning contra la planta en Python (rápido: 2 posturas, J1 y J2)."""

import math

import numpy as np
import pytest

from conftest import requiere_pinocchio

pytestmark = requiere_pinocchio


def test_autotune_identifica_y_termina(cargar):
    from control_adrc_rover.nucleo.autotune import Autotune
    from control_adrc_rover.nucleo.controlador import Controlador
    from control_adrc_rover.nucleo.modelo import ModeloBrazo
    from control_adrc_rover.nucleo.planta import Planta
    extra = {'por_defecto': {'encoder': 'absoluto'},
             'seguridad': {'watchdog_congelar': 0.05,
                           'watchdog_quitar': 0.5},
             'autotune': {'repeticiones': 1, 'paso_wn': 1.6, 'wn_max': 14.0,
                          'posturas': {
                              'retraida': {'Joint_2': -1.5707,
                                           'Joint_3': 2.2},
                              'media': {'Joint_2': -1.0, 'Joint_3': 1.0}}}}
    cfg = cargar(None, montadas=3, extra=extra)
    ctrl = Controlador(cfg, ModeloBrazo(cfg.urdf, cfg), log=lambda s: None)
    arm_real = np.array([0.6, 1.0, 0.84])
    k_real = np.array([3.7485, 3.9375, 3.9375]) * np.array([0.9, 1.1, 1.0])
    q0 = {'Joint_1': math.pi, 'Joint_2': -1.0, 'Joint_3': 1.0}
    planta = Planta(cfg.urdf, ['Joint_1', 'Joint_2', 'Joint_3'],
                    {'Joint_4': 'ausente', 'Joint_5': 'ausente'}, q0,
                    escala_masa={'Joint_2': 1.2, 'Joint_3': 1.2},
                    armadura=arm_real, k_tau=k_real,
                    i0=np.array([0.25, 0.497, 0.497]), retardo_muestras=2)
    at = None
    t, dt = 0.0, ctrl.dt_nominal
    while t < 400.0:
        raw = planta.medir() - ctrl.offset[:3]
        mando = ctrl.paso(t) if ctrl.medir(raw, t) else None
        if at is None and t > 0.3:
            at = Autotune(ctrl, ['Joint_1', 'Joint_2'], log=lambda s: None)
            at.iniciar()
        if at is not None and at.terminado:
            break
        corr = mando.valores[:3] if mando is not None else np.zeros(3)
        planta.avanzar(corr, dt)
        t += dt
    assert at.exito, at.mensaje
    r2 = at.resultados['Joint_2']
    # inercia total "vista" (K_hat/K_real)·(alfa M + J_a) ≈ la identificada
    assert 0.6 < r2['alfa'] < 1.6
    assert 0.6 < r2['beta'] <= 1.0
    assert len(r2['polos']) == 4
    assert ctrl.observador is None
    assert ctrl.factor_limite == pytest.approx(np.ones(3))
