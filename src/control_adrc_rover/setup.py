import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'control_adrc_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='medina',
    maintainer_email='medina@todo.todo',
    description='Control ADRC del brazo del rover (tesis de maestría)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'adrc_node = control_adrc_rover.adrc_node:main',
            'adrc_simple = control_adrc_rover.adrc_simple_node:main',
            'actuador_sim = control_adrc_rover.actuador_sim_node:main',
        ],
    },
)
