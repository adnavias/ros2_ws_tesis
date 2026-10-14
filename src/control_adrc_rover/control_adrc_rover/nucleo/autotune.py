"""Rutina de autotuning por articulación (sin ROS).

Se ejecuta dentro del lazo: el controlador llama a `paso(datos)` en cada
ciclo (vía `Controlador.observador`) y la rutina, escrita como generador,
manda objetivos y espera a que terminen. Etapas:

    1. Identificación. En cada postura (retraída, media, extendida) y
       para cada articulación elegida: movimientos pequeños ±amplitud con
       corriente reducida (`factor_corriente` del límite). Se guardan
       K_hat·i, q*'', q*', M_ii(q) y g_i(q) del URDF y se identifican por
       mínimos cuadrados alfa, J_a, fricción y alfa_g (identificacion.py).
    2. kappa y beta. Con el modelo ya calibrado kappa ≈ 1; beta se elige
       para cubrir la incertidumbre de alfa (elegir_beta).
    3. Ancho de banda. En la postura media se sube wn por pasos
       (`paso_wn`) con movimientos de `amplitud_wn_deg`; en cuanto aparece
       sobreimpulso u oscilación (mismo criterio que el supervisor) se
       detiene y se deja wn = (1 - margen_wn) · wn_que_falló.

Seguridad: se aborta (y se restauran las ganancias y límites originales)
si el error supera `error_aborto_deg`, si el control sale de "habilitado"
(paro, falla, watchdog) o si la corriente se mantiene saturada.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

from .identificacion import Muestras, elegir_beta, identificar
from .supervisor import analizar


class AbortoAutotune(Exception):
    """Se interrumpe la rutina (el motivo va en el mensaje)."""


class Autotune:
    """Máquina de estados del autotuning sobre un Controlador."""

    def __init__(self, ctrl, articulaciones: Optional[List[str]] = None,
                 log=print) -> None:
        """Args:
            articulaciones: nombres a calibrar (por defecto, todas las que
                tienen modelo). Deben ser activas con corriente_con_modelo.
        """
        self.ctrl = ctrl
        self.cfg = ctrl.cfg.autotune
        self.log = log
        modelo_nombres = ctrl.modelo.nombres if ctrl.modelo else []
        if not modelo_nombres:
            raise ValueError('El autotuning necesita el modelo dinámico y '
                             'al menos una articulación con '
                             '"corriente_con_modelo"')
        self.nombres = list(articulaciones or modelo_nombres)
        for nom in self.nombres:
            if nom not in modelo_nombres:
                raise ValueError(f'{nom} no se puede calibrar: debe estar '
                                 f'activa con mando corriente_con_modelo')
        self.g = [ctrl.nombres_gpi_index(n) for n in self.nombres]
        self.k = [ctrl.k_gpi[g] for g in self.g]
        self.m_index = [list(ctrl.modelo.nombres).index(n)
                        for n in self.nombres]
        self.data = ctrl.modelo.nuevo_data()
        self.progreso = 0.0
        self.etapa = 'preparando'
        self.terminado = False
        self.exito = False
        self.mensaje = ''
        self.resultados: Dict[str, dict] = {}
        self._datos: Optional[dict] = None
        self._guardado = None
        self._rutina = None
        self._sat_seguidas = np.zeros(len(self.g), dtype=int)

    # ------------------------------------------------------------------
    def iniciar(self) -> None:
        """Guarda el estado original y engancha la rutina al lazo."""
        c = self.ctrl
        if c.estado != 'habilitado':
            raise ValueError(f'El control está "{c.estado}"; habilítalo '
                             f'antes del autotuning')
        self._guardado = {
            'sintonias': list(c.gpi.sintonias), 'beta': c.beta.copy(),
            'alfa': c.alfa.copy(), 'armadura': c.armadura.copy(),
            'fc': c.fc.copy(), 'alfa_g': c.alfa_g.copy(),
            'factor_limite': c.factor_limite.copy(),
            'supervisor': c.supervisor.habilitado,
        }
        c.supervisor.habilitado = False
        self._rutina = self._generador()
        c.observador = self.paso

    def cancelar(self, motivo: str = 'cancelado') -> None:
        """Detiene la rutina y restaura todo."""
        if not self.terminado:
            self._terminar(False, motivo, restaurar=True)

    def paso(self, datos: dict) -> None:
        """Llamado por el controlador en cada ciclo."""
        if self.terminado:
            return
        self._datos = datos
        try:
            self._vigilar(datos)
            next(self._rutina)
        except StopIteration:
            pass
        except AbortoAutotune as exc:
            self._terminar(False, f'Autotuning abortado: {exc}',
                           restaurar=True)
        except Exception as exc:  # noqa: BLE001 - nunca romper el lazo
            self._terminar(False, f'Autotuning abortado por error: {exc}',
                           restaurar=True)

    # ------------------------------------------------------------------
    def _vigilar(self, d: dict) -> None:
        c = self.ctrl
        if c.estado != 'habilitado':
            raise AbortoAutotune(f'el control pasó a "{c.estado}" '
                                 f'({c.motivo})')
        if d['e'] is None:
            return
        e = np.asarray(d['e'])
        lim = math.radians(float(self.cfg['error_aborto_deg']))
        if np.any(np.abs(e) > lim):
            j = int(np.argmax(np.abs(e)))
            raise AbortoAutotune(
                f'error de {math.degrees(e[j]):.1f}° en '
                f'{c.nombres[c.k_gpi[j]]} (límite '
                f'{self.cfg["error_aborto_deg"]}°)')
        sat = np.asarray(d['saturado'])[self.g]
        self._sat_seguidas = np.where(sat, self._sat_seguidas + 1, 0)
        if np.any(self._sat_seguidas > int(0.5 / c.dt_nominal)):
            raise AbortoAutotune('corriente saturada por más de 0.5 s')

    def _terminar(self, exito: bool, mensaje: str, restaurar: bool) -> None:
        c = self.ctrl
        gd = self._guardado
        if gd is not None:
            c.factor_limite[:] = gd['factor_limite']
            c.supervisor.habilitado = gd['supervisor']
            if restaurar:
                c.beta[:] = gd['beta']
                c.alfa[:] = gd['alfa']
                c.armadura[:] = gd['armadura']
                c.fc[:] = gd['fc']
                c.alfa_g[:] = gd['alfa_g']
                for g in range(c.gpi.m):
                    c.gpi.fijar_sintonia(g, gd['sintonias'][g])
        if c.observador == self.paso:
            c.observador = None
        if c.estado == 'habilitado' and c.traj.active:
            c.traj.fijar(c.q_d)          # se queda donde va
        self.terminado = True
        self.exito = exito
        self.mensaje = mensaje
        self.etapa = 'terminado' if exito else 'abortado'
        self.progreso = 1.0 if exito else self.progreso
        self.log(mensaje)

    # ------------------------------------------------------------------
    # Primitivas de la rutina
    # ------------------------------------------------------------------
    def _t(self) -> float:
        return float(self._datos['t'])

    def _mover(self, objetivo_urdf: Dict[str, float], pct: float):
        """Mueve a una postura (URDF) y espera a que termine."""
        c = self.ctrl
        urdf = c.objetivo.copy()
        usar = np.zeros(c.n, dtype=bool)
        for nom, val in objetivo_urdf.items():
            if nom in c.nombres:
                k = c.nombres.index(nom)
                urdf[k] = float(val)
                usar[k] = True
        usr = np.where(usar, c.a_usuario(urdf), np.nan)
        ok, msg, tr = c.planear(usr, pct)
        if not ok:
            raise AbortoAutotune(msg)
        c.aplicar_trayectoria(tr)
        yield
        while c.traj.active:
            yield

    def _esperar(self, segundos: float):
        t0 = self._t()
        while self._t() - t0 < segundos:
            yield

    # ------------------------------------------------------------------
    def _generador(self):
        c = self.ctrl
        cfg = self.cfg
        posturas = cfg.get('posturas') or {}
        if not posturas:
            posturas = {'actual': {}}
        amp = math.radians(float(cfg['amplitud_deg']))
        pct = float(cfg['velocidad_pct'])
        reps = int(cfg['repeticiones'])
        n_post = len(posturas)
        muestras = {nom: Muestras() for nom in self.nombres}
        total = n_post * len(self.nombres) * reps + len(self.nombres) * 6
        hecho = 0

        # --- 1. Identificación ---
        for nom_post, postura in posturas.items():
            self.etapa = f'identificación: postura {nom_post}'
            yield from self._mover(postura, pct)
            yield from self._esperar(1.0)
            for nom, g, k, mi in zip(self.nombres, self.g, self.k,
                                     self.m_index):
                centro = float(c.q_d[k])
                self._reducir_corriente(g)
                for _ in range(reps):
                    for destino in (centro + amp, centro - amp, centro):
                        gen = self._mover({nom: destino}, pct)
                        for _ in gen:
                            self._grabar(muestras[nom], g, k, mi)
                            yield
                        for _ in self._esperar(0.3):
                            self._grabar(muestras[nom], g, k, mi)
                            yield
                    hecho += 1
                    self.progreso = 0.7 * hecho / total
                c.factor_limite[g] = self._guardado['factor_limite'][g]
                self.etapa = (f'identificación: postura {nom_post}, {nom} '
                              f'lista')

        # --- 2. Mínimos cuadrados, kappa y beta ---
        self.etapa = 'calculando parámetros'
        yield
        k_obj = float(cfg['k_objetivo'])
        calibradas = []
        for nom, g in zip(self.nombres, self.g):
            arm_nom = float(c.armadura[g]) if c.armadura[g] > 0 else (
                c.cfg.articulacion(nom).armadura_nominal)
            if not c.cfg.lazo['incluir_armadura']:
                arm_nom = 0.0
            try:
                res = identificar(muestras[nom], arm_nom)
            except ValueError as exc:
                # una articulación mal identificada no tumba a las demás:
                # se deja como estaba y se reporta
                self.resultados[nom] = {
                    'error': str(exc),
                    'nota': 'sin cambios (datos insuficientes o ruidosos; '
                            'típico de una articulación muy ligera)'}
                self.log(f'Autotune {nom}: {exc}; se deja sin cambios')
                continue
            calibradas.append(nom)
            beta = elegir_beta(res.alfa, res.sigma_alfa, k_obj)
            alfa_anterior = float(c.alfa[g])
            c.alfa[g] = res.alfa
            c.armadura[g] = res.armadura
            c.alfa_g[g] = res.alfa_gravedad
            c.fc[g] = res.friccion_tau / c.k_tau[g] if c.es_corriente[g] \
                else res.friccion_tau
            c.beta[g] = beta
            self.resultados[nom] = {
                'alfa': round(res.alfa, 4), 'armadura': round(res.armadura, 4),
                'friccion_coulomb': round(float(c.fc[g]), 4),
                'alfa_gravedad': round(res.alfa_gravedad, 4),
                'beta': round(beta, 3),
                'sigma_alfa': round(res.sigma_alfa, 4),
                'residuo_rms_Nm': round(res.residuo_rms, 4),
                'alfa_anterior': round(alfa_anterior, 4),
                'muestras': res.n_muestras,
                'nota': res.mensaje,
            }
            self.log(f'Autotune {nom}: {self.resultados[nom]}')

        # --- 3. Ancho de banda ---
        media = posturas.get('media', next(iter(posturas.values())))
        self.etapa = 'ancho de banda: postura media'
        yield from self._mover(media, pct)
        yield from self._esperar(0.5)
        amp_w = math.radians(float(cfg['amplitud_wn_deg']))
        paso_wn = float(cfg['paso_wn'])
        wn_max = float(cfg['wn_max'])
        margen = float(cfg['margen_wn'])
        dt = c.dt_nominal
        for nom, g, k in zip(self.nombres, self.g, self.k):
            if nom not in calibradas:
                continue
            # Se conserva la forma (relación entre polos, o ζ y ε) y sólo
            # se mueve el ancho de banda wn.
            forma = c.gpi.sintonias[g]
            wn = forma.wn_efectivo()
            wn_falla = None
            while True:
                try:
                    forma.con_wn(wn).validar(dt)
                except ValueError:
                    break
                c.gpi.fijar_wn(g, wn)
                self.etapa = f'ancho de banda: {nom}, wn = {wn:.1f} rad/s'
                centro = float(c.q_d[k])
                fallo = False
                for destino in (centro + amp_w, centro):
                    q_ini = float(c.q[k])
                    yield from self._mover({nom: destino}, 100.0)
                    ventana = []
                    t0 = self._t()
                    while self._t() - t0 < c.supervisor.ventana:
                        ventana.append(float(c.q[k]))
                        yield
                    r = analizar(nom, np.array(ventana), q_ini, destino,
                                 c.supervisor.banda)
                    if (r.sobreimpulso_pct > c.supervisor.umbral_pct
                            or r.cruces > c.supervisor.cruces_max):
                        fallo = True
                        break
                hecho += 1
                self.progreso = min(0.99, 0.7 + 0.3 * hecho / total)
                if fallo:
                    wn_falla = wn
                    break
                if wn * paso_wn > wn_max:
                    break
                wn *= paso_wn
            base = wn_falla if wn_falla is not None else min(wn, wn_max)
            wn_final = max(float(c.supervisor.wn_min), (1.0 - margen) * base)
            c.gpi.fijar_sintonia(g, forma.con_wn(wn_final))
            final = c.gpi.sintonias[g]
            c.wn_inicial[g] = final.wn_efectivo()
            self.resultados[nom]['polos'] = [round(float(p), 3)
                                             for p in final.polos]
            self.resultados[nom]['sintonia'] = final.describir()
            self.resultados[nom]['wn_oscilacion'] = (
                None if wn_falla is None else round(wn_falla, 2))
            self.log(f'Autotune {nom}: wn final {wn_final:.2f} rad/s '
                     f'(oscila en {wn_falla})')
        sin = [n for n in self.nombres if n not in calibradas]
        if not calibradas:
            raise AbortoAutotune('ninguna articulación se pudo identificar')
        texto = 'Autotuning terminado'
        if sin:
            texto += f' (sin cambios en {", ".join(sin)}: ver resultados)'
        self._terminar(True, texto, restaurar=False)
        yield

    def _reducir_corriente(self, g: int) -> None:
        """Límite reducido para la articulación excitada: la corriente de
        sostener la postura más `factor_corriente` del margen restante."""
        c = self.ctrl
        f = float(self.cfg['factor_corriente'])
        lim = float(c.limite[g])
        hold = 0.0
        sel = list(c.g_mod)
        if g in sel:
            _, g_cal = c._modelo_cal(c.q, self.data)
            hold = abs(float(g_cal[sel.index(g)])) / float(c.k_tau[g]) + \
                float(c.fc[g])
        reducido = min(lim, hold + f * max(lim - hold, 0.0))
        c.factor_limite[g] = reducido / lim

    def _grabar(self, m: Muestras, g: int, k: int, mi: int) -> None:
        """Agrega una muestra de identificación de la articulación."""
        c = self.ctrl
        d = self._datos
        if d is None or d['cmd'] is None or d['saturado'][g]:
            return
        cmd = float(d['cmd'][g])
        kh = float(c.k_tau[g]) if c.es_corriente[g] else 1.0
        q_mod = d['q'][c.k_gpi[c.g_mod]]
        m_q, g_q = c.modelo.calcular(q_mod, self.data)
        if c.friccion_velocidad:
            fric = math.tanh(float(d['qd_d'][k]) / 0.02)
        else:
            fric = math.tanh(float(c.fc_gain[g]) * float(d['cmd_core'][g]))
        m.agregar(kh * cmd, d['qdd_d'][k], fric, m_q[mi, mi], g_q[mi])
