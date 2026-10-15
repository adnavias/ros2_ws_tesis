import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'control_adrc_rover'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'brazos'),
            glob(os.path.join('config', 'brazos', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'plataformas'),
            glob(os.path.join('config', 'plataformas', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'ajustes'),
            glob(os.path.join('config', 'ajustes', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adán Medina Covarrubias',
    maintainer_email='adan.medina.c.13@gmail.com',
    description='Control ADRC/GPI del brazo del rover (tesis de maestría)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'adrc_simple = control_adrc_rover.control_node:main',
            'actuador_sim = control_adrc_rover.actuador_sim_node:main',
            'estimar_modo = control_adrc_rover.estimar_modo_node:main',
        ],
    },
)
