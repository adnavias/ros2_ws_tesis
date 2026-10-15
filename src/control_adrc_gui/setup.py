import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'control_adrc_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['web/*']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'),
            glob(os.path.join(package_name, 'web', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Adán Medina Covarrubias',
    maintainer_email='adan.medina.c.13@gmail.com',
    description='Interfaz web del control del brazo',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'control_gui = control_adrc_gui.gui_node:main',
        ],
    },
)
