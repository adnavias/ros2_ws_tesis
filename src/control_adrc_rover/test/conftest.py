"""Configuración común de las pruebas (no necesitan ROS en ejecución)."""

import os
import sys

import pytest

AQUI = os.path.dirname(os.path.abspath(__file__))
PAQUETE = os.path.dirname(AQUI)
SRC = os.path.dirname(PAQUETE)
sys.path.insert(0, PAQUETE)

PERFIL = os.path.join(PAQUETE, 'config', 'brazos', 'principal.yaml')
SIM = os.path.join(PAQUETE, 'config', 'plataformas', 'sim.yaml')
REAL = os.path.join(PAQUETE, 'config', 'plataformas', 'real.yaml')


def buscar_paquete(nombre):
    """Sustituto de get_package_share_directory que usa el repositorio."""
    return os.path.join(SRC, nombre)


def hay_pinocchio():
    try:
        import pinocchio  # noqa: F401
        return True
    except ImportError:
        return False


requiere_pinocchio = pytest.mark.skipif(not hay_pinocchio(),
                                        reason='Pinocchio no instalado')


@pytest.fixture
def cargar():
    """Fábrica: cargar(plataforma='sim', montadas=0, estados=None, extra=None)."""
    from control_adrc_rover.nucleo import config as C

    def _cargar(plataforma='sim', montadas=0, estados=None, extra=None):
        capas = [C.leer_yaml(PERFIL)]
        if plataforma:
            capas.append(C.leer_yaml(SIM if plataforma == 'sim' else REAL))
        if extra:
            capas.append(extra)
        return C.construir(capas, [], montadas, estados,
                           buscar_paquete=buscar_paquete)
    return _cargar
