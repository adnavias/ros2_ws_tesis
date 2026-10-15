"""Rutas de los archivos de configuración del paquete.

Autor: Adán Medina Covarrubias
"""

import os

PAQUETE = 'control_adrc_rover'


def dir_config() -> str:
    """Carpeta config/ del paquete.

    Con `colcon build --symlink-install` los YAML de share/ son enlaces a
    src/; en ese caso se devuelve la carpeta de src para que lo que se
    guarde (autotune_<brazo>.yaml) quede en el repositorio.
    """
    from ament_index_python.packages import get_package_share_directory
    share = os.path.join(get_package_share_directory(PAQUETE), 'config')
    marca = os.path.join(share, 'brazos', 'principal.yaml')
    if os.path.islink(marca) or os.path.islink(os.path.dirname(marca)):
        return os.path.dirname(os.path.dirname(os.path.realpath(marca)))
    return share


def ruta_capa(valor: str, sub: str) -> str:
    """Nombre corto (config/<sub>/<valor>.yaml) o ruta directa a un YAML."""
    if not valor:
        return ''
    if valor.endswith('.yaml') or os.sep in valor:
        return os.path.expanduser(valor)
    return os.path.join(dir_config(), sub, f'{valor}.yaml')


def ruta_ajustes(nombre_brazo: str, plataforma: str = '') -> str:
    """config/ajustes/<brazo>_<plataforma>.yaml.

    Cada plataforma (sim, real, ...) guarda sus propias ganancias: lo que
    se afina en Gazebo no se aplica al robot real y viceversa. Sin
    plataforma devuelve el archivo viejo, config/autotune_<brazo>.yaml.
    """
    if not plataforma:
        return os.path.join(dir_config(), f'autotune_{nombre_brazo}.yaml')
    return os.path.join(dir_config(), 'ajustes',
                        f'{nombre_brazo}_{plataforma}.yaml')
