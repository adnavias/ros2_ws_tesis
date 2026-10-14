# Estado completo del control del brazo (lo publica adrc_simple en
# ~/estado a la frecuencia de diagnóstico). Los arreglos por articulación
# siguen el orden de `nombres` (sólo articulaciones activas).
std_msgs/Header header

string brazo                 # perfil cargado
string plataforma            # sim | real | ...
string estado                # esperando_medicion | deshabilitado | habilitado | paro | falla | inactivo
string motivo                # por qué está en ese estado
string watchdog              # ok | congelar | quitar
bool paro_emergencia         # paro virtual activo (latched)
bool modo_calibracion        # la interfaz exige heartbeat
bool trayectoria_activa
bool autotune_activo
float64 autotune_progreso    # 0..1
string autotune_etapa
float64 rampa_gravedad       # 0..1

# Todas las articulaciones del perfil y su estado
string[] nombres_perfil
string[] estados_perfil      # activa | bloqueada | ausente

# Articulaciones activas
string[] nombres
string[] mando               # corriente_con_modelo | torque_sin_modelo | posicion_directa
float64[] posicion_deg       # medida (coordenadas del usuario)
float64[] referencia_deg     # trayectoria deseada
float64[] error_deg
float64[] mando_valor        # A, N·m o posición enviada
float64[] limite_mando
float64[] phi_hat            # perturbación estimada [rad/s^2]
float64[] kappa              # M_ctrl/M_real estimado (NaN = sin datos)
float64[] wn                 # ancho de banda de referencia [rad/s]
float64[] polos              # -Re de los 4 polos por articulación, aplanado
float64[] polos_imag         # Im de los 4 polos (0 si son reales), aplanado
string[] sintonia            # polos | zeta ("" si no usa GPI)
float64[] wn_sintonia        # vista (ζ, wn, ε): wn del par lento [rad/s]
float64[] zeta               # amortiguamiento del par lento
float64[] eps                # wn / w_observador
float64[] zeta_observador    # 0 = igual a zeta
float64[] beta
float64[] alfa
float64[] armadura
float64[] alfa_gravedad
float64[] friccion_coulomb
float64[] sobreimpulso_deg   # del último movimiento
uint32[] saturaciones        # contador
bool[] saturado              # en este instante

float64 velocidad_pct_defecto
bool supervisor_habilitado
string shaper_tipo
float64 shaper_frecuencia_hz
float64 shaper_amortiguamiento

string[] eventos             # últimos eventos (supervisor, fallas, etc.)
