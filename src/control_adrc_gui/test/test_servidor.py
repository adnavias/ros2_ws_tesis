"""Pruebas del servidor web sin ROS (puente falso)."""

import json
import math
import os
import sys
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control_adrc_gui.servidor import ServidorWeb  # noqa: E402


class PuenteFalso:
    def __init__(self):
        self.comandos = []

    def estado(self):
        return {'estado': 'habilitado', 'kappa': [1.0, math.nan]}

    def comando(self, accion, datos):
        self.comandos.append((accion, datos))
        return accion != 'malo', f'hecho {accion}', {'eco': datos.get('x')}


def _servidor():
    p = PuenteFalso()
    s = ServidorWeb(p, '127.0.0.1', 0, periodo_stream=0.01)
    s.iniciar()
    return p, s, f'http://127.0.0.1:{s.puerto}'


def test_estado_json_sin_nan():
    _, s, url = _servidor()
    try:
        datos = json.loads(urllib.request.urlopen(url + '/api/estado').read())
        assert datos == {'estado': 'habilitado', 'kappa': [1.0, None]}
    finally:
        s.detener()


def test_comando_y_pagina():
    p, s, url = _servidor()
    try:
        req = urllib.request.Request(
            url + '/api/comando', data=json.dumps({'accion': 'paro',
                                                   'x': 3}).encode(),
            headers={'Content-Type': 'application/json'})
        r = json.loads(urllib.request.urlopen(req).read())
        assert r == {'ok': True, 'mensaje': 'hecho paro', 'eco': 3}
        assert p.comandos[0][0] == 'paro'
        html = urllib.request.urlopen(url + '/').read().decode()
        assert 'PARO' in html
        js = urllib.request.urlopen(url + '/web/app.js').read().decode()
        assert 'EventSource' in js
    finally:
        s.detener()


def test_stream_envia_eventos():
    _, s, url = _servidor()
    try:
        resp = urllib.request.urlopen(url + '/api/stream', timeout=2)
        linea = resp.readline().decode()
        assert linea.startswith('data: ')
        assert json.loads(linea[6:])['estado'] == 'habilitado'
        resp.close()
    finally:
        s.detener()


def test_ruta_fuera_de_web_no_se_sirve():
    _, s, url = _servidor()
    try:
        try:
            urllib.request.urlopen(url + '/web/../servidor.py')
            assert False, 'no debió servir el archivo'
        except urllib.error.HTTPError as exc:
            assert exc.code == 404
    finally:
        s.detener()
