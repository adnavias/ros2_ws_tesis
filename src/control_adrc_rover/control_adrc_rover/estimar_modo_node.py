"""Herramienta: estima el modo de vibración de la base a partir de phi_hat.

Escucha /adrc_simple/estado durante unos segundos (por ejemplo justo
después de un movimiento rápido del brazo), toma la perturbación estimada
phi_hat de cada articulación y busca el pico del espectro:

    ros2 run control_adrc_rover estimar_modo --segundos 8
    ros2 run control_adrc_rover estimar_modo --segundos 8 --articulacion Joint_1

Imprime la frecuencia [Hz] y el amortiguamiento sugeridos para el
conformador (shaper.frecuencia_hz, shaper.amortiguamiento). El servicio
/adrc_simple/estimar_modo hace lo mismo con el búfer interno a 200 Hz.

Autor: Adán Medina Covarrubias
"""

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node

from control_adrc_interfaces.msg import EstadoControl
from control_adrc_rover.nucleo.shaper import estimar_modo


def main(args=None) -> None:
    """Graba phi_hat y estima el modo dominante."""
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--segundos', type=float, default=8.0)
    p.add_argument('--articulacion', default='')
    p.add_argument('--topico', default='/adrc_simple/estado')
    p.add_argument('--fmin', type=float, default=0.8)
    p.add_argument('--fmax', type=float, default=20.0)
    opciones, resto = p.parse_known_args(sys.argv[1:] if args is None else args)
    rclpy.init(args=resto)
    nodo = Node('estimar_modo')
    datos = []

    def cb(msg: EstadoControl) -> None:
        st = msg.header.stamp
        datos.append((st.sec + st.nanosec * 1e-9, list(msg.nombres),
                      list(msg.phi_hat)))
    nodo.create_subscription(EstadoControl, opciones.topico, cb, 50)
    print(f'Grabando phi_hat de {opciones.topico} por {opciones.segundos} s '
          f'(mueve el brazo ahora o justo antes)...')
    t0 = time.monotonic()
    while time.monotonic() - t0 < opciones.segundos:
        rclpy.spin_once(nodo, timeout_sec=0.05)
    nodo.destroy_node()
    rclpy.shutdown()
    if len(datos) < 64:
        print(f'Muy pocos datos ({len(datos)}); ¿está corriendo el control?')
        return
    t = np.array([d[0] for d in datos])
    nombres = datos[-1][1]
    phi = np.array([d[2] for d in datos], dtype=float)
    fs = 1.0 / max(float(np.median(np.diff(t))), 1e-4)
    print(f'{len(datos)} muestras a {fs:.1f} Hz')
    mejor = None
    for k, n in enumerate(nombres):
        if opciones.articulacion and n != opciones.articulacion:
            continue
        r = estimar_modo(phi[:, k], fs, opciones.fmin, opciones.fmax)
        if r is None:
            print(f'{n}: sin pico claro')
            continue
        print(f'{n}: f = {r["frecuencia_hz"]:.2f} Hz, ζ = '
              f'{r["amortiguamiento"]:.3f} ({r["metodo"]}, resolución '
              f'{r["resolucion_hz"]:.2f} Hz)')
        if mejor is None or r['amplitud'] > mejor['amplitud']:
            mejor = r
    if mejor:
        print(f'\nSugerencia para el conformador:\n  ros2 param set /adrc_simple '
              f'shaper.frecuencia_hz {mejor["frecuencia_hz"]:.2f}\n  ros2 param '
              f'set /adrc_simple shaper.amortiguamiento '
              f'{max(mejor["amortiguamiento"], 0.0):.3f}\n  ros2 param set '
              f'/adrc_simple shaper.tipo zvd')


if __name__ == '__main__':
    main()
