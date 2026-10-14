"""Conformadores de entrada (input shaping) y estimación del modo de
vibración de la base.

Base elástica. El brazo va montado sobre el chasis del rover, que con la
suspensión y las llantas se comporta como un modo poco amortiguado
(frecuencia f, amortiguamiento ζ). Convolucionar la referencia con un
tren de impulsos que se cancela a sí mismo en ese modo evita excitarlo:

    q*_c(t) = Σ_k A_k q*(t - t_k),   Σ A_k = 1.

Con ωd = 2π f sqrt(1-ζ²),  Td = 2π/ωd  y  K = exp(-ζπ / sqrt(1-ζ²)):

    ZV   A = [1, K] / (1+K),                t = [0, Td/2]
    ZVD  A = [1, 2K, K²] / (1+K)²,           t = [0, Td/2, Td]
    EI   A = [(1+V)/4, (1-V)/2, (1+V)/4],    t = [0, Td/2, Td]
         (V = vibración residual tolerada; forma para ζ pequeño)

El conformado retrasa el movimiento Td/2 (ZV) o Td (ZVD, EI). Como la
convolución es lineal, la velocidad y la aceleración conformadas son las
mismas sumas sobre q*' y q*''; el feedforward sigue siendo exacto.

La implementación guarda la referencia cruda en un búfer circular y
evalúa la suma con interpolación lineal; así funciona igual si el
objetivo cambia a medio movimiento.

Estimación del modo. Tras un movimiento, la perturbación estimada por el
GPI (phi_hat) contiene la oscilación de la base. `estimar_modo` toma el
espectro (ventana de Hann, interpolación parabólica del pico) en una banda
y estima ζ con el ancho a media potencia. La misma función sirve después
para la aceleración de una IMU.

Autor: Adán Medina Covarrubias
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional, Tuple

import numpy as np


def impulsos(tipo: str, frecuencia_hz: float, zeta: float,
             tolerancia_ei: float = 0.05) -> Tuple[np.ndarray, np.ndarray]:
    """Amplitudes y tiempos [s] del conformador; ('ninguno') -> [1], [0]."""
    if tipo == 'ninguno' or frecuencia_hz <= 0.0:
        return np.array([1.0]), np.array([0.0])
    zeta = float(np.clip(zeta, 0.0, 0.95))
    raiz = math.sqrt(1.0 - zeta ** 2)
    td = 1.0 / (frecuencia_hz * raiz)
    k = math.exp(-zeta * math.pi / raiz)
    if tipo == 'zv':
        a = np.array([1.0, k]) / (1.0 + k)
        t = np.array([0.0, td / 2.0])
    elif tipo == 'zvd':
        a = np.array([1.0, 2.0 * k, k * k]) / (1.0 + k) ** 2
        t = np.array([0.0, td / 2.0, td])
    elif tipo == 'ei':
        v = float(np.clip(tolerancia_ei, 0.0, 0.3))
        a = np.array([(1.0 + v) / 4.0, (1.0 - v) / 2.0, (1.0 + v) / 4.0])
        t = np.array([0.0, td / 2.0, td])
    else:
        raise ValueError(f'Conformador desconocido: {tipo}')
    return a, t


class ReferenciaConformada:
    """Aplica el conformador a (q*, q*', q*'') con un búfer de historia."""

    def __init__(self, tipo: str = 'ninguno', frecuencia_hz: float = 2.0,
                 zeta: float = 0.05, tolerancia_ei: float = 0.05) -> None:
        """Configura el conformador; 'ninguno' deja pasar la referencia."""
        self.configurar(tipo, frecuencia_hz, zeta, tolerancia_ei)

    def configurar(self, tipo: str, frecuencia_hz: float, zeta: float,
                   tolerancia_ei: float = 0.05) -> None:
        """Cambia el conformador (vacía la historia)."""
        self.tipo = tipo
        self.frecuencia_hz = float(frecuencia_hz)
        self.zeta = float(zeta)
        self.tolerancia_ei = float(tolerancia_ei)
        self.a, self.t = impulsos(tipo, frecuencia_hz, zeta, tolerancia_ei)
        self.retardo = float(self.t[-1])
        self._hist: deque = deque()

    @property
    def activo(self) -> bool:
        """True si hay conformado (más de un impulso)."""
        return self.a.size > 1

    def reiniciar(self) -> None:
        """Olvida la historia (p. ej. al rehabilitar)."""
        self._hist.clear()

    def _en(self, t: float):
        """Referencia cruda interpolada en el tiempo t."""
        h = self._hist
        if t <= h[0][0]:
            return h[0][1]
        if t >= h[-1][0]:
            return h[-1][1]
        # búsqueda desde el final (los retardos son cortos)
        for i in range(len(h) - 1, 0, -1):
            t0, x0 = h[i - 1]
            if t0 <= t:
                t1, x1 = h[i]
                w = (t - t0) / max(t1 - t0, 1e-12)
                return x0 + w * (x1 - x0)
        return h[0][1]

    def aplicar(self, t: float, q: np.ndarray, qd: np.ndarray,
                qdd: np.ndarray):
        """Guarda la referencia cruda en t y devuelve la conformada."""
        if not self.activo:
            return q, qd, qdd
        x = np.concatenate([q, qd, qdd])
        h = self._hist
        if h and t <= h[-1][0]:
            h[-1] = (h[-1][0], x)
        else:
            h.append((t, x))
        while len(h) > 2 and h[1][0] < t - self.retardo - 0.05:
            h.popleft()
        y = np.zeros_like(x)
        for a_k, t_k in zip(self.a, self.t):
            y += a_k * self._en(t - t_k)
        n = q.size
        return y[:n], y[n:2 * n], y[2 * n:]


def duracion_minima_modo(frecuencia_hz: float, ciclos: float) -> float:
    """Duración mínima de un movimiento ligada al periodo del modo."""
    if frecuencia_hz <= 0.0 or ciclos <= 0.0:
        return 0.0
    return float(ciclos) / float(frecuencia_hz)


def estimar_modo(senal: np.ndarray, fs: float, f_min: float = 0.8,
                 f_max: float = 20.0) -> Optional[dict]:
    """Frecuencia [Hz] y amortiguamiento del pico dominante en la banda.

    Returns:
        dict con frecuencia_hz, amortiguamiento, amplitud, o None si la
        señal es muy corta o no hay pico claro.
    """
    x = np.asarray(senal, dtype=float)
    x = x[np.isfinite(x)]
    if x.size < 32 or fs <= 0.0:
        return None
    # quita la tendencia lineal (movimiento lento, gravedad)
    t = np.arange(x.size)
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    if np.allclose(x, 0.0):
        return None
    n_fft = int(2 ** math.ceil(math.log2(x.size * 8)))
    ventana = np.hanning(x.size)
    espectro = np.abs(np.fft.rfft(x * ventana, n_fft))
    f = np.fft.rfftfreq(n_fft, 1.0 / fs)
    banda = (f >= f_min) & (f <= min(f_max, fs / 2.0))
    if banda.sum() < 3:
        return None
    idx_banda = np.nonzero(banda)[0]
    k = idx_banda[np.argmax(espectro[banda])]
    if k <= 0 or k >= espectro.size - 1:
        return None
    # interpolación parabólica del pico (en log)
    y0, y1, y2 = np.log(espectro[k - 1:k + 2] + 1e-300)
    den = y0 - 2.0 * y1 + y2
    delta = 0.5 * (y0 - y2) / den if abs(den) > 1e-12 else 0.0
    f0 = float(f[k] + delta * (f[1] - f[0]))
    pico = espectro[k]
    # el pico debe destacar del resto de la banda (si no, es ruido)
    prominencia = float(pico / max(np.median(espectro[banda]), 1e-300))
    if prominencia < 6.0:
        return None
    # ancho a media potencia (-3 dB)
    nivel = pico / math.sqrt(2.0)
    i1 = k
    while i1 > 0 and espectro[i1] > nivel:
        i1 -= 1
    i2 = k
    while i2 < espectro.size - 1 and espectro[i2] > nivel:
        i2 += 1
    ancho = float(f[i2] - f[i1])
    # La ventana y la duración finita ensanchan el pico: se resta el ancho
    # propio de la ventana de Hann (≈ 1.44 fs / N).
    ancho_ventana = 1.44 * fs / x.size
    ancho_modo = max(ancho - ancho_ventana, 0.0)
    zeta = ancho_modo / (2.0 * f0) if f0 > 0.0 else float('nan')
    metodo = 'media_potencia'
    # Si la señal es un decaimiento libre, el decremento logarítmico de los
    # picos (tras filtrar en banda alrededor de f0) es más preciso.
    zeta_dec = _decremento_logaritmico(x, fs, f0)
    if zeta_dec is not None:
        zeta, metodo = zeta_dec, 'decremento_logaritmico'
    return {'frecuencia_hz': f0, 'amortiguamiento': float(zeta),
            'amplitud': float(2.0 * pico / ventana.sum()),
            'resolucion_hz': float(ancho_ventana), 'metodo': metodo,
            'prominencia': prominencia}


def _decremento_logaritmico(x: np.ndarray, fs: float,
                            f0: float) -> Optional[float]:
    """ζ por ajuste lineal de ln|picos| de la señal filtrada en banda."""
    n = x.size
    espectro = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    espectro[(f < 0.6 * f0) | (f > 1.6 * f0)] = 0.0
    y = np.fft.irfft(espectro, n)
    # picos positivos
    idx = np.nonzero((y[1:-1] > y[:-2]) & (y[1:-1] >= y[2:]))[0] + 1
    idx = idx[y[idx] > 0.0]
    if idx.size < 4:
        return None
    # descarta extremos (efecto de borde del filtro)
    idx = idx[1:-1]
    amp = y[idx]
    if np.any(amp <= 0.0):
        return None
    t = idx / fs
    pendiente = np.polyfit(t, np.log(amp), 1)[0]     # = -ζ ωn
    if pendiente >= 0.0:
        return None
    wn = 2.0 * math.pi * f0
    zeta = -pendiente / wn
    # ajuste deficiente si la amplitud casi no decae (señal forzada)
    if amp[0] / amp[-1] < 1.3:
        return None
    return float(zeta)
