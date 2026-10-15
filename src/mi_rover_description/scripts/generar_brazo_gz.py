#!/usr/bin/env python3
"""Genera urdf/brazo_gz.urdf.xacro a partir de urdf/brazo.urdf.

El xacro generado es el mismo brazo con dos argumentos para Gazebo:

    montadas      N (1..5): sólo se incluyen Joint_1..Joint_N y sus
                  eslabones; el resto no existe (brazo parcialmente
                  montado, como en las pruebas reales).
    escala_masa   factor de masa e inercia de TODOS los eslabones móviles
                  (pruebas de robustez del control: 0.5, 1.5, ...).

Uso (si cambias brazo.urdf, p. ej. al exportar de nuevo de SolidWorks):
    python3 scripts/generar_brazo_gz.py

Autor: Adán Medina Covarrubias
"""

import os
import re
import xml.etree.ElementTree as ET

AQUI = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(AQUI, '..', 'urdf', 'brazo.urdf')
SALIDA = os.path.join(AQUI, '..', 'urdf', 'brazo_gz.urdf.xacro')


def main() -> None:
    arbol = ET.parse(URDF)
    raiz = arbol.getroot()
    # número de cada articulación/eslabón (Joint_k -> Link_k)
    nivel = {}
    for j in raiz.findall('joint'):
        m = re.match(r'Joint_(\d+)$', j.get('name', ''))
        if m:
            k = int(m.group(1))
            nivel[('joint', j.get('name'))] = k
            nivel[('link', j.find('child').get('link'))] = k
    partes = ['<?xml version="1.0"?>',
              '<!-- GENERADO por scripts/generar_brazo_gz.py a partir de '
              'brazo.urdf. No editar a mano. -->',
              '<robot xmlns:xacro="http://ros.org/wiki/xacro" name="brazo_2">',
              '  <xacro:arg name="montadas" default="5"/>',
              '  <xacro:arg name="escala_masa" default="1.0"/>',
              '  <xacro:property name="montadas" value="$(arg montadas)"/>',
              '  <xacro:property name="em" value="$(arg escala_masa)"/>']
    for elem in list(raiz):
        k = nivel.get((elem.tag, elem.get('name')))
        if elem.tag == 'link' and k is not None:
            ine = elem.find('inertial')
            if ine is not None:
                masa = ine.find('mass')
                masa.set('value', f'${{{float(masa.get("value"))} * em}}')
                inr = ine.find('inertia')
                for c in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
                    inr.set(c, f'${{{float(inr.get(c))} * em}}')
        texto = ET.tostring(elem, encoding='unicode').strip()
        texto = '\n'.join('  ' + linea for linea in texto.splitlines())
        if k is not None:
            partes.append(f'  <xacro:if value="${{montadas >= {k}}}">')
            partes.append(texto)
            partes.append('  </xacro:if>')
        else:
            partes.append(texto)
    partes.append('</robot>')
    with open(SALIDA, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(partes) + '\n')
    print(f'Escrito {os.path.normpath(SALIDA)}')


if __name__ == '__main__':
    main()
