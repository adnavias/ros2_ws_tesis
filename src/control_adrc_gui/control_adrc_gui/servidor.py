"""Servidor web de la interfaz (sólo biblioteca estándar de Python).

No depende de ROS: recibe un objeto "puente" con dos métodos,

    puente.estado()                  -> dict serializable a JSON
    puente.comando(accion, datos)    -> (ok: bool, mensaje: str, extra: dict)

y expone:

    GET  /                 página (web/index.html)
    GET  /web/<archivo>    archivos estáticos (js, css)
    GET  /api/estado       estado actual (JSON)
    GET  /api/stream       Server-Sent Events con el estado (~20 Hz)
    POST /api/comando      {"accion": "...", ...} -> {"ok", "mensaje", ...}
    POST /api/heartbeat    latido del navegador (modo calibración)

Se usa ThreadingHTTPServer: cada navegador conectado tiene su hilo.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import json
import math
import mimetypes
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

DIR_WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')


def _limpiar(obj):
    """Convierte NaN/inf en None para que el JSON sea válido."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _limpiar(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_limpiar(v) for v in obj]
    return obj


class _Manejador(BaseHTTPRequestHandler):
    """Atiende una petición HTTP."""

    server_version = 'ControlBrazo/1.0'
    puente = None            # se asigna al crear el servidor
    periodo_stream = 0.05

    def log_message(self, formato, *args):  # silencio en la terminal
        return

    # ------------------------------------------------------------------
    def _json(self, datos, codigo: int = 200) -> None:
        cuerpo = json.dumps(_limpiar(datos), ensure_ascii=False).encode()
        self.send_response(codigo)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(cuerpo)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(cuerpo)

    def _archivo(self, ruta: str) -> None:
        ruta = os.path.normpath(ruta)
        if not ruta.startswith(DIR_WEB) or not os.path.isfile(ruta):
            self.send_error(404, 'No encontrado')
            return
        tipo = mimetypes.guess_type(ruta)[0] or 'application/octet-stream'
        with open(ruta, 'rb') as fh:
            cuerpo = fh.read()
        self.send_response(200)
        self.send_header('Content-Type', tipo + (
            '; charset=utf-8' if tipo.startswith('text/') or
            tipo.endswith('javascript') else ''))
        self.send_header('Content-Length', str(len(cuerpo)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(cuerpo)

    # ------------------------------------------------------------------
    def do_GET(self) -> None:  # noqa: N802 - nombre de BaseHTTPRequestHandler
        """Página, estáticos, estado y stream."""
        ruta = self.path.split('?', 1)[0]
        if ruta in ('/', '/index.html'):
            self._archivo(os.path.join(DIR_WEB, 'index.html'))
        elif ruta.startswith('/web/'):
            self._archivo(os.path.join(DIR_WEB, ruta[len('/web/'):]))
        elif ruta == '/api/estado':
            self._json(self.puente.estado())
        elif ruta == '/api/stream':
            self._stream()
        else:
            self.send_error(404, 'No encontrado')

    def do_POST(self) -> None:  # noqa: N802
        """Comandos y heartbeat."""
        ruta = self.path.split('?', 1)[0]
        largo = int(self.headers.get('Content-Length') or 0)
        crudo = self.rfile.read(largo) if largo > 0 else b'{}'
        try:
            datos = json.loads(crudo.decode() or '{}')
        except (ValueError, UnicodeDecodeError):
            self._json({'ok': False, 'mensaje': 'JSON inválido'}, 400)
            return
        if ruta == '/api/heartbeat':
            ok, msg, extra = self.puente.comando('heartbeat', datos)
            self._json({'ok': ok})
            return
        if ruta != '/api/comando':
            self.send_error(404, 'No encontrado')
            return
        accion = str(datos.get('accion', ''))
        try:
            ok, msg, extra = self.puente.comando(accion, datos)
        except Exception as exc:  # noqa: BLE001 - se reporta a la página
            ok, msg, extra = False, f'Error: {exc}', {}
        respuesta = {'ok': ok, 'mensaje': msg}
        respuesta.update(extra or {})
        self._json(respuesta)

    def _stream(self) -> None:
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        try:
            while True:
                datos = json.dumps(_limpiar(self.puente.estado()),
                                   ensure_ascii=False)
                self.wfile.write(f'data: {datos}\n\n'.encode())
                self.wfile.flush()
                time.sleep(self.periodo_stream)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


class ServidorWeb:
    """Servidor HTTP en un hilo propio."""

    def __init__(self, puente, host: str = '0.0.0.0', puerto: int = 8080,
                 periodo_stream: float = 0.05) -> None:
        """Crea el servidor (no lo arranca)."""
        manejador = type('Manejador', (_Manejador,),
                         {'puente': puente,
                          'periodo_stream': float(periodo_stream)})
        self.httpd = ThreadingHTTPServer((host, int(puerto)), manejador)
        self.httpd.daemon_threads = True
        self.hilo: Optional[threading.Thread] = None

    @property
    def puerto(self) -> int:
        """Puerto real (útil si se pidió el 0)."""
        return self.httpd.server_address[1]

    def iniciar(self) -> None:
        """Arranca el hilo del servidor."""
        self.hilo = threading.Thread(target=self.httpd.serve_forever,
                                     daemon=True)
        self.hilo.start()

    def detener(self) -> None:
        """Detiene el servidor."""
        self.httpd.shutdown()
        self.httpd.server_close()
