"""Pruebas de carga y validación de la configuración."""

import pytest

from control_adrc_rover.nucleo import config as C


def test_perfil_sim_carga(cargar):
    cfg = cargar('sim')
    assert cfg.nombres == ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4',
                           'Joint_5']
    j1 = cfg.articulacion('Joint_1')
    assert j1.encoder == 'absoluto'
    assert j1.limite_mando == pytest.approx(6.54)
    assert j1.k_tau == pytest.approx(3.7485, rel=1e-3)
    assert j1.armadura_nominal == pytest.approx(0.777, rel=1e-2)
    assert cfg.lazo['incluir_armadura'] is False
    assert cfg.articulacion('Joint_5').mando == 'posicion_directa'


def test_real_como_el_brazo_real(cargar):
    cfg = cargar('real')
    assert all(a.encoder == 'absoluto' for a in cfg.articulaciones)
    j1, j2 = cfg.articulacion('Joint_1'), cfg.articulacion('Joint_2')
    assert j1.signo == -1.0 and j1.limite_mando == pytest.approx(5.0)
    assert j1.sintonia.modo == 'zeta' and j1.sintonia.wn == 0.5
    assert j2.sintonia.eps == pytest.approx(0.2)
    assert j2.limite_mando * j2.k_tau <= 27.0 + 1e-6
    assert cfg.crudo['articulaciones']['Joint_2']['topico_mando'] == \
        '/arm_teleop/joint2/current'
    assert j2.postura_encendido_urdf == pytest.approx(-1.5707)


def test_real_encoder_incremental_con_un_cambio(cargar):
    cfg = cargar('real', extra={'articulaciones': {
        'Joint_1': {'encoder': 'incremental'}}})
    assert cfg.articulacion('Joint_1').encoder == 'incremental'
    assert cfg.articulacion('Joint_2').encoder == 'absoluto'


def test_montadas_1_deja_solo_j1(cargar):
    cfg = cargar('real', montadas=1)
    assert [a.nombre for a in cfg.movibles] == ['Joint_1']
    assert all(a.estado == 'ausente' for a in cfg.articulaciones[1:])


def test_estados_explicitos(cargar):
    cfg = cargar('sim', estados={'Joint_4': 'bloqueada',
                                 'Joint_5': 'ausente'})
    assert cfg.articulacion('Joint_4').estado == 'bloqueada'
    assert [a.nombre for a in cfg.movibles] == [
        'Joint_1', 'Joint_2', 'Joint_3']


def test_limite_por_encima_del_reductor_se_rechaza(cargar):
    with pytest.raises(C.ErrorConfig, match='par continuo'):
        cargar('sim', extra={'articulaciones': {
            'Joint_2': {'limite_mando': 7.35}}})


def test_limite_por_encima_de_corriente_continua(cargar):
    with pytest.raises(C.ErrorConfig, match='corriente continua'):
        cargar('sim', extra={'articulaciones': {
            'Joint_1': {'limite_mando': 9.0}}})


def test_polo_demasiado_rapido(cargar):
    with pytest.raises(C.ErrorConfig, match=r'\|λ\|·dt'):
        cargar('sim', extra={'por_defecto': {'polos': [10.0, 10.0, 150.0,
                                                       150.0]}})


def test_polos_desde_wn():
    assert C.polos_desde({'wn': 5.0, 'relacion_observador': 4.0}) == \
        [5.0, 5.0, 20.0, 20.0]


def test_sintonia_por_defecto_es_polos(cargar):
    st = cargar('sim').articulacion('Joint_2').sintonia
    assert st.modo == 'polos'
    assert list(st.polos) == pytest.approx([8.0, 8.0, 32.0, 32.0])
    assert (st.wn, st.zeta, st.eps) == pytest.approx((8.0, 1.0, 0.25))


def test_sintonia_zeta_por_articulacion(cargar):
    cfg = cargar('sim', extra={'articulaciones': {'Joint_3': {
        'sintonia': 'zeta', 'wn': 6.0, 'zeta': 0.9, 'eps': 0.2}}})
    st = cfg.articulacion('Joint_3').sintonia
    assert st.modo == 'zeta' and st.wn == pytest.approx(6.0)
    assert not st.polos_reales()
    assert cfg.articulacion('Joint_2').sintonia.modo == 'polos'


@pytest.mark.parametrize('malo', [{'sintonia': 'lqr'},
                                  {'sintonia': 'zeta', 'eps': 2.0},
                                  {'sintonia': 'zeta', 'zeta': 0.0}])
def test_sintonia_invalida(cargar, malo):
    with pytest.raises(C.ErrorConfig):
        cargar('sim', extra={'articulaciones': {'Joint_2': malo}})


def test_parsear_estados():
    assert C.parsear_estados('Joint_2=ausente, Joint_3=bloqueada') == {
        'Joint_2': 'ausente', 'Joint_3': 'bloqueada'}
    with pytest.raises(C.ErrorConfig):
        C.parsear_estados('Joint_2=rota')


def test_sin_activas_se_rechaza(cargar):
    with pytest.raises(C.ErrorConfig, match='ninguna articulación activa'):
        cargar('sim', estados={n: 'ausente' for n in
                               ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4',
                                'Joint_5']})
