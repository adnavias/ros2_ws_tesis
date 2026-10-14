# Guía de uso — Control del brazo del rover (versión 2)

Paquetes ROS 2: `control_adrc_rover`, `control_adrc_interfaces`, `control_adrc_gui`
Autor del sistema: Ing. Adán Medina Covarrubias

Esta guía está pensada para alguien que nunca ha visto el código. Explica qué hace el sistema, cómo arrancarlo (en simulación y en el robot), cómo mandarle posiciones al brazo, cómo usar la interfaz web, qué entra y qué sale, cómo se configura y qué hacer si algo falla. No hace falta saber Python para usarlo.

## Contenido

1. ¿Qué es esto? (resumen en un minuto)
2. Qué cambió respecto a la versión anterior
3. Conceptos mínimos de ROS 2
4. Cómo fluye la información
5. Archivos del sistema y dónde están
6. Instalación y compilación
7. Uso en simulación, paso a paso
8. Cómo mandar posiciones al brazo
9. La interfaz web
10. Qué entra y qué sale (tópicos, servicios, acciones, parámetros)
11. Qué hace cada programa
12. Configuración: perfil, plataforma y ajustes
13. El control por dentro: polos, kappa, beta y protecciones
14. Robot real (Jetson)
15. Autotuning y supervisor
16. Base elástica: conformador y estimación del modo
17. Pruebas automáticas
18. Problemas comunes
19. Reglas de seguridad
20. Valores supuestos que conviene verificar
21. Glosario

---

## 1. ¿Qué es esto? (resumen en un minuto)

El control sirve para cualquier brazo serial de hasta 5 articulaciones descrito por un URDF. El brazo principal tiene:

| Art. | Función | Actuador | Mando |
|---|---|---|---|
| J1 | Base giratoria | Yellow Jacket + sinfín | corriente [A] |
| J2 | Hombro | Maxon EC 60 flat + GSW 62 A 100:1 | corriente [A] |
| J3 | Codo | Maxon EC 60 flat + GSW 62 A 100:1 | corriente [A] |
| J4 | Muñeca | Yellow Jacket + sinfín | corriente [A] |
| J5 | Actuador final | Servo de rotación continua | posición directa |

Tú le dices al brazo "ve a estos ángulos" (en grados). El sistema:

1. Calcula una trayectoria suave (polinomio de 9.º grado) desde donde va hasta donde quieres.
2. Mide la posición de cada articulación (200 veces por segundo).
3. Con un compensador GPI (Sira-Ramírez, familia ADRC) calcula la aceleración necesaria y la convierte en corriente con el modelo del brazo, i = K⁻¹ (β·M(q)·v + g(q)).
4. Limita la corriente para proteger motores y reductores y publica el mando.

Cada articulación puede estar **activa** (se controla), **bloqueada** (fija, su masa cuenta) o **ausente** (no está montada, su masa no cuenta). Así funciona igual con el brazo completo, con sólo J1 y J2, o con sólo J1.

Además hay una **interfaz web** opcional que corre en paralelo. El control funciona completo sin ella.

## 2. Qué cambió respecto a la versión anterior

| Antes | Ahora |
|---|---|
| Arreglos fijos de 5 valores en el YAML | Configuración por articulación; n sale del perfil |
| El modelo siempre incluía las 5 masas | Estados activa/bloqueada/ausente; modelo reducido con Pinocchio |
| `brazo_adrc.yaml` / `brazo_real.yaml` | Perfil del brazo + plataforma (sim/real) + ajustes guardados |
| zeta del par dominante | Los 4 polos reales del GPI, configurables |
| Polos [10, 10, 50, 50] (aprox.) | Polos por defecto [8, 8, 32, 32]: más margen de robustez |
| Sin watchdog de medición | Watchdog 20/100 ms (sim 50/500 ms) |
| Anti-windup sólo en s1 | Back-calculation en s1 + integración condicional en s2 |
| Gravedad completa al arrancar | Integradores inicializados en gravedad y feedforward en rampa |
| dt del reloj de pared | dt del timestamp de la medición (sigue a Gazebo) |
| Lazo y replaneo en el mismo hilo | Lazo, watchdog y servicios en grupos de callbacks separados |
| Sin límites articulares | Paredes virtuales y recorte de objetivos con los límites del URDF |
| — | Sanidad del encoder (NaN, saltos, vueltas) |
| Mismo M(q) para controlar y verificar | Cota baja (β·M) para controlar y cota alta para verificar trayectorias |
| — | Lifecycle node, servicios, acciones, paro latched, heartbeat |
| — | Supervisor que baja wn si hay sobreimpulso; autotuning |
| — | Conformador ZV/ZVD/EI y estimación del modo de la base |
| — | Interfaz web (sin instalar nada) |
| J2/J3 límite 7.35 A | 6.85 A: con K_tau = 3.94 N·m/A, 7.35 A daba 28.9 N·m > 27 N·m del reductor |

Hallazgo importante (sección 13.3): el rango de k = M_modelo/M_real en el que el lazo es estable no es sólo k < 2. Con la medición retrasada 1–2 muestras es aproximadamente **0.15 < k < 3**, y **bien amortiguado sólo para 0.7 ≲ k ≲ 1.3**. Un modelo que subestima la inercia también oscila, y bajar wn no lo corrige.

## 3. Conceptos mínimos de ROS 2

- **Nodo**: un programa. Aquí: `adrc_simple` (control), `actuador_sim` (sólo simulación), `control_gui` (interfaz web).
- **Tópico**: canal con nombre por el que un nodo publica y otro escucha.
- **Servicio**: pregunta-respuesta (habilitar, paro, guardar…).
- **Acción**: tarea larga con progreso (mover a, autotuning).
- **Parámetro**: valor de configuración; algunos se cambian en vivo (`ros2 param set`).
- **Lifecycle node**: nodo con estados (sin configurar → inactivo → activo). El control se configura y activa solo al arrancar.
- **`source`**: cada terminal debe cargar ROS y el workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 4. Cómo fluye la información

```text
SIMULACIÓN (Gazebo)

 Tú / interfaz web / otro sistema
     │ objetivo [°] (tópico, acción MoverA o página web)
     ▼
 ┌──────────────────────────────┐   EstadoControl, paro latched
 │ adrc_simple (control)        │──────────────────────────► control_gui ──► navegador
 │ trayectoria → GPI → β·M(q) → │
 │ límites → mando              │
 └──────────────────────────────┘
     │ corriente [A] (J1–J4), posición (J5)
     ▼
 actuador_sim  (corriente → torque, servo simulado)
     │ torque [N·m]
     ▼
 ros_gz_bridge ◄──► Gazebo ──► posición [rad] ──► adrc_simple

ROBOT REAL (Jetson): igual, pero los drivers publican la posición y
reciben el mando directamente; actuador_sim no se lanza.
```

El nodo de control es el mismo en ambos casos; sólo cambia la plataforma (`sim:=true/false`).

## 5. Archivos del sistema y dónde están

```text
~/ros2_ws/src/
├── mi_rover_description/
│   ├── urdf/brazo.urdf                 Brazo completo (lo usa el modelo del control)
│   ├── urdf/brazo_gz.urdf.xacro        NUEVO: brazo para Gazebo con montadas y escala_masa
│   ├── urdf/rover_completo.urdf.xacro  Rover + brazo + plugins (acepta montadas)
│   ├── scripts/generar_brazo_gz.py     NUEVO: regenera brazo_gz.urdf.xacro
│   └── launch/gazebo.launch.py         Gazebo + puente ros_gz_bridge (montadas, escala_masa)
│
├── control_adrc_interfaces/            NUEVO (ament_cmake)
│   ├── msg/EstadoControl.msg
│   ├── srv/CargarPerfil.srv, GuardarConfiguracion.srv
│   └── action/MoverA.action, Autotune.action
│
├── control_adrc_rover/
│   ├── config/brazos/principal.yaml    PERFIL del brazo (física, motores, ganancias)
│   ├── config/plataformas/sim.yaml     Lo que cambia en simulación
│   ├── config/plataformas/real.yaml    Lo que cambia en la Jetson (# <-- AJUSTAR)
│   ├── config/autotune_principal.yaml  Se crea al guardar/autotunear (opcional)
│   ├── launch/brazo.launch.py          NUEVO: un comando para todo
│   ├── launch/brazo_sim.launch.py      Compatibilidad (mismo comando de antes)
│   ├── control_adrc_rover/
│   │   ├── control_node.py             Nodo de control (lifecycle "adrc_simple")
│   │   ├── actuador_sim_node.py        Motores simulados (generalizado a n)
│   │   ├── estimar_modo_node.py        Herramienta para el modo de la base
│   │   ├── rutas.py                    Rutas de config
│   │   └── nucleo/                     Lógica sin ROS (se prueba con pytest)
│   │       ├── config.py               Carga y validación por capas
│   │       ├── modelo.py               M(q), g(q) con articulaciones ausentes/bloqueadas
│   │       ├── gpi.py                  Compensador GPI, polos, anti-windup, phi_hat
│   │       ├── trayectoria.py          Polinomio de 9.º grado
│   │       ├── seguridad.py            Encoder, watchdog, paredes, pendiente
│   │       ├── shaper.py               Conformador y estimación del modo
│   │       ├── supervisor.py           Sobreimpulso, reducción de wn, kappa en línea
│   │       ├── identificacion.py       Mínimos cuadrados (autotuning)
│   │       ├── autotune.py             Rutina de autotuning
│   │       ├── controlador.py          Une todo
│   │       └── planta.py               Brazo simulado en Python (pruebas)
│   └── test/                           pytest (sin ROS), Monte Carlo
│
└── control_adrc_gui/                   NUEVO: interfaz web
    └── control_adrc_gui/
        ├── gui_node.py                 Nodo puente (sólo interfaces públicas)
        ├── servidor.py                 Servidor HTTP (biblioteca estándar)
        └── web/index.html, app.js, app.css
```

**Regla de oro:** para usar y ajustar el sistema sólo se editan los YAML de `config/`. Los `.py` no se tocan.

Archivos que se ELIMINAN: `config/brazo_adrc.yaml`, `config/brazo_real.yaml`, `control_adrc_rover/adrc_simple_node.py`, `control_adrc_rover/trayectoria.py` y `control_adrc_rover/modelo_dinamico.py` (su contenido vive ahora en `nucleo/`).

## 6. Instalación y compilación

No hay que instalar nada nuevo: todo usa ROS 2 Humble, numpy, PyYAML, Pinocchio (ya requerido) y la biblioteca estándar de Python. El paquete de interfaces sólo necesita `rosidl_default_generators`, que viene con ROS 2 (también en `ros-humble-ros-base` de la Jetson).

Como cambiaron archivos y paquetes, la primera vez compila desde cero:

```bash
cd ~/ros2_ws
rm -rf build/control_adrc_rover install/control_adrc_rover
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`control_adrc_interfaces` se compila primero automáticamente. Con `--symlink-install` los YAML se editan y basta relanzar; los ajustes que guardes desde la interfaz quedan en `src/control_adrc_rover/config/`.

## 7. Uso en simulación, paso a paso

Dos terminales (con los `source` hechos):

**Terminal 1 — Gazebo (ya incluye el puente):**

```bash
ros2 launch mi_rover_description gazebo.launch.py
```

Si ya corres tu propio `parameter_bridge`, usa `puente:=false` para no duplicarlo.

**Terminal 2 — Control + motores simulados + interfaz web:**

```bash
ros2 launch control_adrc_rover brazo.launch.py
```

Se abre el navegador de Windows en `http://localhost:8080`. Mensajes esperados:

```text
Modelo dinámico: .../brazo.urdf | coordenadas libres ['Joint_1', ..., 'Joint_4']
Configurado: brazo "principal" | Joint_1=activa/corriente_con_modelo, ...
Cero fijado en la postura actual: [...] rad (URDF)
Control habilitado
Interfaz web lista: http://localhost:8080 ...
```

Variantes:

```bash
ros2 launch control_adrc_rover brazo.launch.py gui:=false          # sólo el control
ros2 launch control_adrc_rover brazo_sim.launch.py                 # comando de antes (sin interfaz)
```

**Sólo J1 en simulación** (como la prueba real): Gazebo y el control deben coincidir.

```bash
ros2 launch mi_rover_description gazebo.launch.py montadas:=1
ros2 launch control_adrc_rover brazo.launch.py montadas:=1
```

**Prueba de robustez**: Gazebo con masas distintas a las del modelo del control:

```bash
ros2 launch mi_rover_description gazebo.launch.py escala_masa:=1.4
```

**El cero**: al arrancar, la postura actual es (0, 0, …). Los ángulos que mandas son relativos a ella (`lazo.referencia_objetivos: relativa_arranque`). Para usar ángulos absolutos del URDF pon `"urdf"`.

## 8. Cómo mandar posiciones al brazo

### 8.1 Comando principal (grados)

```bash
ros2 topic pub --once /brazo/objetivo_deg std_msgs/msg/Float64MultiArray "{data: [90.0, 30.0, -45.0, 0.0, 0.0]}"
```

- Puedes mandar un valor por cada articulación del **perfil** (5 aquí; las no activas se ignoran) o uno por cada articulación **activa** (con `montadas:=1`, basta `{data: [45.0]}`).
- Un valor extra al final es el % de velocidad: `{data: [0.0, 135.0, -135.0, 0.0, 0.0, 20.0]}`.
- `.nan` deja esa articulación en su objetivo actual.
- Siempre con punto decimal y con `--once`.

### 8.2 Por nombres

```bash
ros2 topic pub --once /brazo/objetivo_nombres_deg sensor_msgs/msg/JointState "{name: ['Joint_2'], position: [45.0], velocity: [30.0]}"
```

`position` va en grados; `velocity[0]` (opcional) es el %.

### 8.3 Acción MoverA (con progreso y resultado)

```bash
ros2 action send_goal --feedback /adrc_simple/mover_a control_adrc_interfaces/action/MoverA "{nombres: ['Joint_1'], posiciones_deg: [30.0], velocidad_pct: 40.0}"
```

Devuelve el error final y el sobreimpulso medido. Cancelarla deja el brazo donde va.

### 8.4 Radianes (compatibilidad)

```bash
ros2 topic pub --once /target_joint_states sensor_msgs/msg/JointState "{name: ['Joint_2'], position: [0.5]}"
```

### 8.5 Velocidad

- Rango 1 % a 200 %; 100 % = la mitad de la velocidad física (`factor_100pct: 0.5`).
- La duración se calcula sola y todas las articulaciones llegan a la vez.
- Si la trayectoria pediría más corriente de la permitida (con la cota ALTA del modelo), se alarga: "alargada para no exceder la corriente".
- Si sostener alguna postura del camino ya excede el límite, el objetivo se rechaza.
- Cambiar la velocidad por defecto en vivo: `ros2 param set /adrc_simple velocidad_pct_defecto 30.0`.

### 8.6 Límites articulares

Los objetivos fuera de los límites del URDF (menos `margen_limite`) se recortan (o se rechazan con `objetivo_fuera_de_rango: rechazar`). Cerca de los límites actúa una pared virtual suave.

## 9. La interfaz web

### 9.1 Cómo abrirla

- **Simulación (WSL):** `ros2 launch control_adrc_rover brazo.launch.py` abre sola `http://localhost:8080`.
- **Robot:** en la Jetson `ros2 launch control_adrc_rover brazo.launch.py sim:=false`. En la terminal aparece la dirección, p. ej. `http://192.168.1.20:8080`; ábrela en la computadora conectada a la red del rover. No hace falta ROS en esa computadora, sólo un navegador.
- Funciona sin internet (no carga nada de fuera). Puerto: `puerto:=8090`.

### 9.2 Qué tiene

- **PARO** grande, siempre visible (también con la tecla Espacio). Queda trabado hasta "Liberar paro"; después hay que "Habilitar".
- Estado: habilitado/deshabilitado/paro/falla, watchdog, saturación, modo calibración y el motivo de la última falla.
- Gráficas en vivo de la articulación elegida: posición real vs. trayectoria deseada, error y mando con sus límites.
- Mover: objetivo en grados por articulación, % de velocidad, "Mover sólo ésta", ir a cero, detener.
- Perfil y articulaciones: elegir brazo y poner cada articulación activa/bloqueada/ausente (reinicia el control y fija el cero).
- Ganancias: sintonía por polos (λ1..λ4) o por ζ, ω<sub>n</sub>, ε (selector por articulación; la otra vista muestra el equivalente), β, α, armadura, α_g, fricción y límite por articulación, con kappa estimado, sobreimpulso y saturaciones. Cada cambio lo valida el control; si no es válido aparece el motivo.
- Conformador: tipo, frecuencia y amortiguamiento; botón "Estimar modo".
- Autotuning: articulaciones, aplicar/guardar, progreso y resultados.
- Guardar configuración al YAML y lista de eventos.

### 9.3 Modo calibración y heartbeat

Al marcar "Modo calibración" el control exige que la página siga abierta: el navegador manda un latido cada 0.2 s y el nodo de la interfaz publica `/brazo/interfaz/heartbeat`. Si se cierra la pestaña o se cae la red por más de `heartbeat_timeout` (0.5 s real, 1.0 s sim), el control pasa a **paro**. Fuera del modo calibración la interfaz no es necesaria para nada.

### 9.4 Por qué web y no rqt/PyQt

La PC no necesita ROS ni que el DDS cruce la red (WSL detrás de NAT y routers Wi-Fi suelen bloquear el multicast). La web usa una sola conexión TCP, funciona con Wi-Fi del rover, cable directo o hotspot, y sirve aunque la Jetson no tenga pantalla.

## 10. Qué entra y qué sale

Prefijo `~` = `/adrc_simple/`.

**Entradas (tópicos)**

| Tópico | Tipo | Unidades |
|---|---|---|
| `interfaz.topico_estado` (sim: `/world/empty/model/mi_rover/joint_state`) | sensor_msgs/JointState | rad |
| `/brazo/objetivo_deg` | std_msgs/Float64MultiArray | ° (+ %) |
| `/brazo/objetivo_nombres_deg` | sensor_msgs/JointState | ° (velocity[0] = %) |
| `/target_joint_states` | sensor_msgs/JointState | rad |
| `/brazo/paro_emergencia` | std_msgs/Bool | true = paro |
| `/brazo/interfaz/heartbeat` | std_msgs/Empty | (modo calibración) |
| `interfaz.topico_imu` (opcional) | sensor_msgs/Imu | m/s² |

**Salidas**

| Tópico | Tipo | Contenido |
|---|---|---|
| `/brazo/<J>/cmd_actuador` (patrón) | std_msgs/Float64 | J1–J4 corriente [A]; J5 posición [rad o °] |
| `/brazo/cmd_actuador` (formatos únicos) | Float64MultiArray o JointState | igual; en JointState la corriente va en `effort` y la posición del servo en `position` |
| `~/estado` | control_adrc_interfaces/EstadoControl | todo el estado (50 Hz) |
| `/brazo/paro_emergencia/estado` | std_msgs/Bool (latched) | paro activo |
| `~/error_deg/<J>`, `~/state`, `~/reference` | Float64, JointState | diagnóstico heredado |

**Servicios**: `~/habilitar` (std_srvs/SetBool), `~/paro_emergencia` y `~/liberar_paro` (std_srvs/Trigger), `~/modo_calibracion` (SetBool), `~/cargar_perfil` (CargarPerfil), `~/guardar_configuracion` (GuardarConfiguracion), `~/estimar_modo` (Trigger).

**Acciones**: `~/mover_a` (MoverA), `~/autotune` (Autotune).

**Parámetros dinámicos** (se validan antes de aplicarse):

| Parámetro | Rango | Significado |
|---|---|---|
| `<J>.sintonia` | polos, zeta | cómo se eligen las ganancias (sec. 13.1) |
| `<J>.polos` | 4 valores > 0, \|λ\|·dt < 0.5, mínimo ≥ 0.5 | polos del GPI [rad/s]; al cambiarlo pasa a modo polos |
| `<J>.wn`, `<J>.zeta`, `<J>.eps`, `<J>.zeta_observador` | wn > 0.5; ζ 0.2–5; 0 < ε ≤ 1; ζo 0 (= ζ) o 0.2–5 | al cambiar cualquiera pasa a modo zeta; los que no cambies se toman del equivalente actual |
| `<J>.beta` | 0.05–1.5 (avisa si < 0.6) | margen de inercia |
| `<J>.alfa` | 0.05–20 | escala de inercia de eslabones |
| `<J>.armadura` | 0–20 | inercia de rotores [kg·m²] |
| `<J>.alfa_gravedad` | 0–3 | escala de gravedad |
| `<J>.friccion_coulomb` | 0–5 | compensación [A] |
| `<J>.limite_mando` | ≤ corriente continua y ≤ par continuo/K_tau | límite [A] |
| `velocidad_pct_defecto` | 1–200 | % por defecto |
| `shaper.tipo`, `shaper.frecuencia_hz`, `shaper.amortiguamiento`, `shaper.tolerancia_ei` | — | conformador (con el brazo quieto) |
| `supervisor.habilitado` | bool | supervisor |

Ejemplos:

```bash
ros2 param set /adrc_simple Joint_2.polos "[6.0, 6.0, 24.0, 24.0]"
ros2 param set /adrc_simple Joint_2.zeta 1.2       # pasa a modo zeta
ros2 param set /adrc_simple Joint_2.sintonia polos # vuelve a polos (si son reales)
ros2 service call /adrc_simple/paro_emergencia std_srvs/srv/Trigger
ros2 service call /adrc_simple/liberar_paro std_srvs/srv/Trigger
ros2 service call /adrc_simple/habilitar std_srvs/srv/SetBool "{data: true}"
ros2 service call /adrc_simple/cargar_perfil control_adrc_interfaces/srv/CargarPerfil "{brazo: '', montadas: 2, estados: ''}"
ros2 service call /adrc_simple/guardar_configuracion control_adrc_interfaces/srv/GuardarConfiguracion "{archivo: '', incluir_estados: false}"
ros2 topic echo /adrc_simple/estado
```

## 11. Qué hace cada programa

**`control_node.py` (nodo `adrc_simple`)** — En cada medición nueva (200 Hz):

1. Revisa la medición (NaN, saltos, vueltas) y la convierte a convención URDF según el encoder.
2. Evalúa la trayectoria y, si está activo, el conformador.
3. GPI: aceleración virtual v con los 4 polos; paredes virtuales cerca de los límites.
4. Inversa de B: τ = β·M_cal(q)·v + r(t)·g_cal(q) con el modelo reducido (sólo articulaciones activas con modelo; ausentes sin masa; bloqueadas fijas).
5. Corriente = τ/K_tau + compensación de fricción; saturación; límite de pendiente; anti-windup.
6. Publica el mando y, cada 4 ciclos, el estado.

Un temporizador a 100 Hz vigila la medición (watchdog) y el heartbeat. Los objetivos se planean en otro hilo y se aplican de golpe.

**`actuador_sim_node.py`** — Sólo simulación. Lee el mismo perfil y plataforma. Corriente → torque con zona muerta i0; servo J5 simulado con un PD; articulaciones bloqueadas/ausentes que existan en Gazebo se sostienen rígidas. Si no llega mando 0.1 s, torque cero.

**`gui_node.py` (nodo `control_gui`)** — Sirve la página y la conecta con el control usando sólo sus interfaces públicas.

**`estimar_modo`** — Herramienta de línea de comandos (sección 16).

**`brazo.launch.py`** — Argumentos: `brazo`, `sim`, `plataforma`, `montadas`, `estados`, `archivo_ajustes`, `gui`, `abrir_navegador`, `puerto`.

## 12. Configuración: perfil, plataforma y ajustes

### 12.1 Capas

La configuración se arma en capas; cada una sobrescribe a la anterior:

1. `config/brazos/<brazo>.yaml` — el brazo (URDF, motores, articulaciones, ganancias).
2. `config/plataformas/<sim|real>.yaml` — tópicos, encoders, signos, límites de la plataforma.
3. `config/autotune_<brazo>.yaml` — lo guardado desde la interfaz o por el autotuning (si existe).
4. Parámetros `montadas` y `estados` del launch.

En cada capa, `por_defecto` aplica a todas las articulaciones y `articulaciones.<nombre>` a una sola. El orden de `articulaciones` es el orden de los objetivos.

Reglas: números con punto decimal; espacios, no tabuladores; tras editar, relanzar.

### 12.2 Motores

```yaml
motores:
  maxon_ec60_gsw62:
    k_t_caja: 3.9375          # N·m/A a la salida de la caja
    relacion_caja: 100.0
    relacion_sinfin: 1.0
    eficiencia_sinfin: 1.0
    inercia_rotor: 8.404e-5   # kg·m²
    corriente_vacio: 0.497
    corriente_continua: 9.34
    par_continuo_salida: 27.0
```

K_tau = k_t_caja · relacion_sinfin · eficiencia_sinfin; armadura = inercia_rotor · (relacion_caja · relacion_sinfin)². El control rechaza un `limite_mando` mayor que `corriente_continua` o que `par_continuo_salida / K_tau`.

### 12.3 Por articulación

| Clave | Valores | Significado |
|---|---|---|
| `estado` | activa, bloqueada, ausente | ver sección 1 |
| `mando` | corriente_con_modelo, torque_sin_modelo, posicion_directa | tipo de mando |
| `motor` | nombre en `motores` | obligatorio con corriente_con_modelo |
| `limite_mando` | A (o N·m) | protege motor y reductor |
| `sintonia` | polos, zeta | cómo se eligen las ganancias (por defecto polos) |
| `polos` | [λ1, λ2, λ3, λ4] | modo polos (formato viejo: `wn` y `relacion_observador`) |
| `wn`, `zeta`, `eps`, `zeta_observador` | rad/s, —, (0, 1], 0 = ζ | modo zeta |
| `beta` | ≤ 1 | margen de inercia |
| `alfa`, `armadura`, `alfa_gravedad`, `friccion_coulomb` | — | calibración (autotuning) |
| `encoder` | incremental, absoluto | convención de la medición |
| `postura_encendido_urdf` | rad | postura física al encender (incremental) |
| `offset_absoluto` | rad | cero del encoder absoluto respecto al URDF |
| `signo` | 1.0 / -1.0 | invierte medición y mando |
| `nombre_en_estado`, `nombre_en_mando` | texto | nombres en los mensajes del driver |
| `medida` | bool | si el driver reporta su posición |
| `unidad_posicion` | rad, deg | sólo posicion_directa |
| `posicion_bloqueo` | rad | postura si está bloqueada |
| `v_max_fisica`, `a_max_100pct`, `jerk_max_100pct` | rad/s, rad/s², rad/s³ | trayectoria (jerk 0 = sin límite) |
| `margen_limite`, `rigidez_pared`, `amortiguamiento_pared` | — | paredes virtuales |
| `salto_max` | rad | salto creíble por muestra |
| `pendiente_max` | A/s | límite de pendiente del mando (0 = sin límite) |
| `topico_mando` | texto | tópico propio (si no sigue el patrón) |

Reglas de estado: lo que cuelga de una articulación ausente debe estar ausente (se valida con el árbol del URDF y el mensaje lo explica).

### 12.4 Encoders: incremental o absoluto

- **Incremental** (por defecto en el robot): lee 0 al encender, así que q_urdf = signo · q_medida + `postura_encendido_urdf`. Importante: esa postura debe ser la real al encender; con ella el modelo sabe hacia dónde jala la gravedad.
- **Absoluto**: q_urdf = signo · q_medida + `offset_absoluto`.

Para cambiar una articulación basta una línea en `plataformas/real.yaml`:

```yaml
articulaciones:
  Joint_1:
    encoder: "absoluto"
    offset_absoluto: 0.0
```

### 12.5 Secciones globales

- `lazo`: `frecuencia_control`, `referencia_objetivos`, `feedforward_gravedad`, `feedforward_aceleracion`, `rampa_gravedad` (s), `antiwindup_ganancia`, `factor_inercia_sup`, `factor_gravedad_sup`, `margen_corriente_trayectoria`, `incluir_armadura` (false en sim), `modelo_friccion` (zona_muerta | velocidad), `habilitar_al_arrancar`.
- `trayectoria`: `factor_100pct`, `velocidad_pct_defecto`, `velocidad_pct_min`, `duracion_minima`, `objetivo_fuera_de_rango`.
- `seguridad`: `watchdog_congelar`, `watchdog_quitar`, `paro_accion` (sin_corriente | congelar), `heartbeat_timeout`, `muestras_invalidas_max`, `error_max_deg`, `accion_error` (sostener | sin_corriente).
- `shaper`, `supervisor`, `autotune`, `diagnostico`: ver secciones 15 y 16.
- `interfaz` (plataforma): `topico_estado`, `qos_estado_best_effort`, `tipo_mando` (float64_por_articulacion | float64_multiarray | joint_state), `patron_topico_mando` (`{articulacion}` se sustituye), `topico_mando_unico`, `patron_topico_fuerza`, tópicos de objetivos, `topico_heartbeat`, `topico_imu`, `usar_stamp_medicion`.
- `actuador_sim` (plataforma sim): motores "reales" de la planta, servo y sostén.

### 12.6 Agregar otro brazo

1. Copia `config/brazos/principal.yaml` como `config/brazos/<nombre>.yaml`.
2. Cambia `nombre`, `urdf` (ruta o `package://...`) y `articulaciones` (con los nombres del URDF y en orden base → efector).
3. Si la plataforma tiene otros tópicos, crea `config/plataformas/<otra>.yaml`.
4. `ros2 launch control_adrc_rover brazo.launch.py brazo:=<nombre> plataforma:=<otra>`.

Aparece también en la lista de la interfaz.

## 13. El control por dentro: polos, kappa, beta y protecciones

### 13.1 Ley GPI y polinomio característico

Cada articulación se trata como q'' = κ v + φ, con e = q − q*:

```text
s2' = e − k3 s2,   s1' = s2,
v   = q*'' − (k2 s2' + k1 s2 + k0 s1).
```

Con κ = 1 el error cumple p(s) = s⁴ + k3 s³ + k2 s² + k1 s + k0 = (s+λ1)(s+λ2)(s+λ3)(s+λ4). Los cuatro λi se configuran (reales, positivos):

k3 = Σλi, k2 = Σ_{i<j} λiλj, k1 = Σ_{i<j<k} λiλjλk, k0 = Πλi.

Por defecto [8, 8, 32, 32] (wn = 8, observador 4 veces más rápido).

**Dos formas de dar las ganancias** (clave `sintonia` por articulación; en la interfaz, el selector "sintonía"):

- `polos`: los cuatro λi reales directamente.
- `zeta`: se eligen ζ, ω<sub>n</sub> y ε, y el polinomio es

```text
p(s) = (s² + 2 ζ ωn s + ωn²) · (s² + 2 ζo ωp s + ωp²),    ωp = ωn / ε
```

  El primer par (lento) fija cómo se cierra el error de seguimiento; el segundo (rápido) es la parte "observador" que estima la perturbación φ. ε dice cuántas veces más rápido es el observador (ε = 0.25 → 4 veces). ζo es el amortiguamiento del observador; si se deja en 0 vale lo mismo que ζ.

Equivalencias útiles:

| ζ, ε | Polos (−λ) | Comentario |
|---|---|---|
| ζ = 1, ε = 0.25 | ωn, ωn, 4ωn, 4ωn | idéntico al defecto [8, 8, 32, 32] con ωn = 8 |
| ζ > 1 | ωn(ζ ± √(ζ²−1)), … | reales y distintos; el más lento baja (con ζ = 1.5, 0.38 ωn): más amortiguado pero converge más lento |
| ζ < 1 | −ζωn ± j ωn√(1−ζ²), … | complejos: el error puede sobrepasar; la interfaz avisa si ζ < 0.7 |

Al revés, cualquier juego de 4 polos reales λ1 ≤ λ2 ≤ λ3 ≤ λ4 se escribe como ωn = √(λ1λ2), ζ = (λ1+λ2)/(2ωn), ωp = √(λ3λ4), ζo = (λ3+λ4)/(2ωp), ε = ωn/ωp. Por eso la interfaz muestra siempre las dos vistas: la del modo elegido se edita y la otra muestra el equivalente (en gris). De zeta a polos sólo se puede pasar directo si los polos son reales (ζ ≥ 1, ζo ≥ 1); si no, escribe los 4 polos que quieras.

El supervisor y el autotuning conservan la "forma" (la relación entre polos, o ζ y ε) y sólo mueven ωn.

Cómo afectan a la robustez (simulación del doble integrador con 2 muestras de retardo a 200 Hz, ωn = 8; rango de κ = M_ctrl/M_real en que converge):

| Sintonía | κ mínimo | κ máximo |
|---|---|---|
| ζ = 1, ε = 0.25 (defecto) | ~0.2 | ~3 |
| ζ = 0.7, ε = 0.25 | ~0.3 | ~3 |
| ζ = 1.3, ε = 0.25 | ~0.15 | ~3 |
| ζ = 1, ε = 0.5 | ~0.3 | > 5 |
| ζ = 1, ε = 0.15 | ~0.15 | ~2 |

Reglas prácticas: ζ entre 1 y 1.3; ε = 0.25. Si el modelo **subestima** la inercia (κ bajo, oscilación lenta), baja ε (observador más rápido) o sube ζ. Si la **sobreestima** (κ alto, vibración rápida) o hay mucho retardo/ruido, sube ε hacia 0.4–0.5. El polo más rápido debe cumplir |λ|·dt ≤ 0.5 (a 200 Hz, ωn/ε ≤ 100 rad/s).

La perturbación estimada es φ̂ = k0 s1 + (k1 − k2 k3) s2 [rad/s²]. Se publica y sirve para estimar el modo de la base y kappa.

Importante: con un escalón de referencia el GPI sí se pasa (sus ceros); por eso todos los movimientos usan la trayectoria de 9.º grado con feedforward.

### 13.2 Inversa de B y las dos cotas del modelo

```text
M_cal(q) = D M(q) D + diag(J_a),  D = diag(√α)       (calibración)
control:     τ = S M_cal S v + r(t) α_g⊙g(q),  S = diag(√β)   (cota baja)
trayectoria: i = |f_M M_cal q*''|/K + |f_g g| /K + i0 ≤ margen·límite  (cota alta)
```

La cota baja (β ≤ 1) mantiene κ pequeño para la estabilidad; la cota alta (`factor_inercia_sup` 1.25, `factor_gravedad_sup` 1.15) evita trayectorias que saturarían.

### 13.3 Rango de κ = M_ctrl/M_real

Con κ ≠ 1 el polinomio es s³(s+k3) + κ(k2 s² + k1 s + k0). Se analizó el lazo discreto a 200 Hz con 1 y 2 muestras de retardo de medición:

| Polos | Retardo | Estable para κ en | Bien amortiguado (ζ ≳ 0.6) |
|---|---|---|---|
| [8, 8, 32, 32] | 1 muestra | 0.15 – 5.0 | ≈ 0.7 – 1.3 |
| [8, 8, 32, 32] | 2 muestras | 0.16 – 3.1 | ≈ 0.7 – 1.1 |
| [10, 10, 50, 50] | 1 muestra | 0.14 – 3.7 | ≈ 0.7 – 1.1 |
| [10, 10, 50, 50] | 2 muestras | 0.15 – 2.3 | ninguno (ζ máx ≈ 0.58) |

Consecuencias:

- El límite superior (~2–3) explica las oscilaciones de la prueba con J1 y J2: el modelo tenía las masas de J3–J5 y daba κ ≈ 3 en J2 (de 1.0 a 3.2 en J1 según la postura). Con J3–J5 "ausentes", κ = 1. En la planta de pruebas con 15 ms de retardo el caso viejo oscila 16° pico a pico y el corregido no oscila.
- También hay **límite inferior**: con κ < ~0.5 el amortiguamiento cae aunque sea estable, y esto no depende de wn (bajar wn no lo arregla). Por eso β no se baja a ciegas: lo elige el autotuning con la incertidumbre de α.
- Con la medición retrasada, los polos [8, 8, 32, 32] tienen bastante más margen que [10, 10, 50, 50]: en el Monte Carlo (±50 % en masas, armadura y K_tau, 3 muestras de retardo) los primeros no fallaron ningún caso y los segundos fallaron 6 de 27.

### 13.4 Anti-windup

Cuando la corriente satura, Δv = (i_sat − i)·K/M_ii se realimenta a s1 (back-calculation, ganancia `antiwindup_ganancia`·λ1) y s2 se congela si seguiría empujando hacia la saturación.

### 13.5 Arranque suave

Al habilitar, los integradores se inicializan para que la primera corriente sea exactamente la de sostener el peso. Después el feedforward de gravedad entra en rampa (`rampa_gravedad`, 1 s) mientras los integradores le ceden lo mismo, sin salto de corriente.

### 13.6 Tiempo, dt y watchdog

- El tiempo del lazo es el `header.stamp` de la medición: en Gazebo es tiempo simulado, así que el control sigue a la simulación aunque corra más lento que el tiempo real. Si el driver no llena el stamp, se usa la hora de llegada.
- dt = diferencia de stamps (acotado entre 0.2 y 3 veces el nominal). Mediciones repetidas se ignoran; si llegan más rápido que el lazo, se diezman.
- Watchdog (reloj monótono): sin medición más de `watchdog_congelar` → se congelan los integradores y se sostiene la gravedad; más de `watchdog_quitar` → corriente cero y estado **falla** (se sale con Habilitar cuando vuelve la medición).

### 13.7 Otras protecciones

- Encoder: NaN o saltos mayores que `salto_max` se rechazan; más de `muestras_invalidas_max` seguidas → falla. Las articulaciones continuas se desenvuelven.
- Error de seguimiento mayor que `error_max_deg` → con `accion_error: sostener` (por defecto) se aborta el movimiento y se sostiene la postura actual; si vuelve a pasar en menos de 3 s, falla con corriente cero (`sin_corriente` corta de inmediato).
- Paredes virtuales en los límites del URDF.
- Límite de pendiente del mando (`pendiente_max`).
- Paro de emergencia (servicio, tópico, interfaz): latched; `paro_accion: sin_corriente` (por defecto) o `congelar`. Ojo: con corriente cero J2/J3 (reductor strain wave) pueden bajar por su peso; J1/J4 (sinfín) normalmente no.

## 14. Robot real (Jetson)

1. Copiar el workspace (o `git pull`) y compilar (sección 6).
2. Con los drivers encendidos, descubrir los nombres:

```bash
ros2 topic list
ros2 topic info -v <tópico>
ros2 topic echo --once <tópico>
```

3. Editar `config/plataformas/real.yaml`, sólo las líneas `# <-- AJUSTAR`: tópicos, QoS, `tipo_mando`, nombres, `signo`, `unidad_posicion` del servo y el tipo de encoder de cada articulación.
4. Revisar en `config/brazos/principal.yaml` las `postura_encendido_urdf` (postura física al encender).
5. Verificar la frecuencia de la medición: `ros2 topic hz <topico_estado>` (≈ 200 Hz).
6. Lanzar:

```bash
ros2 launch control_adrc_rover brazo.launch.py sim:=false
```

7. Abrir la interfaz en la PC del rover con la dirección que imprime la terminal.

**Prueba con sólo J1** (J2–J5 desmontadas):

```bash
ros2 launch control_adrc_rover brazo.launch.py sim:=false montadas:=1
```

Objetivo de prueba: `{data: [10.0, 5.0]}` (un ángulo + 5 %) o desde la interfaz "Mover sólo ésta". Con J1 y J2: `montadas:=2`.

Antes de la primera prueba, revisa la sección 19.

## 15. Autotuning y supervisor

### 15.1 Autotuning

Desde la interfaz (botón) o:

```bash
ros2 action send_goal --feedback /adrc_simple/autotune control_adrc_interfaces/action/Autotune "{articulaciones: ['Joint_1', 'Joint_2'], aplicar: true, guardar: false}"
```

Etapas:

1. **Identificación**: en las posturas `autotune.posturas` (retraída, media, extendida) mueve cada articulación ±`amplitud_deg` con corriente reducida (la de sostener la postura más `factor_corriente` del margen restante). Con la aceleración de referencia (sin derivar el encoder) resuelve por mínimos cuadrados α, J_a, fricción y α_g. Si M_ii no cambia entre posturas (p. ej. J3, J4), identifica un solo factor de inercia total.
2. **β**: tras calibrar, κ ≈ 1; β cubre la incertidumbre de α (±2σ), sin bajar de 0.6.
3. **Ancho de banda**: en la postura media sube wn por pasos (`paso_wn`) hasta que aparece sobreimpulso u oscilación (o `wn_max`) y deja wn 35 % abajo (`margen_wn`).

Se aborta (y restaura todo) si el error supera `error_aborto_deg`, si el control sale de habilitado o si la corriente queda saturada más de 0.5 s.

Los resultados quedan en `config/autotune_<brazo>.yaml` si se guardan (se carga al arrancar y se edita a mano).

**Antes de correrlo**: verifica que las posturas no choquen con el rover (q2 = 0 es vertical hacia arriba; q2 = −1.5707 horizontal hacia adelante).

### 15.2 Supervisor

Tras cada movimiento observa `supervisor.ventana` segundos. Si el sobreimpulso supera max(`umbral_sobreimpulso_deg`, `umbral_sobreimpulso_pct`·|Δq|) o hay más de `cruces_max` cruces del error, multiplica los polos de esa articulación por `factor_reduccion` (0.85) sin bajar de `wn_min`, y lo registra en los eventos. **Nunca sube ganancias.** Si κ estimado es < 0.6, no baja wn (no ayudaría) y pide el autotuning.

κ se estima en línea con mínimos cuadrados recursivos de φ̂ contra q*'' (sección 13.1); aparece en la interfaz y en `EstadoControl.kappa`.

## 16. Base elástica: conformador y estimación del modo

El chasis con suspensión se comporta como un modo poco amortiguado. El conformador (input shaper) filtra la referencia para no excitarlo:

| Tipo | Retardo | Comentario |
|---|---|---|
| ninguno | 0 | sin filtro |
| zv | Td/2 | sensible a error de frecuencia |
| zvd | Td | más robusto (recomendado) |
| ei | Td | tolera vibración residual V |

La duración mínima de cada movimiento queda ligada al periodo del modo (`ciclos_modo_min`/f).

Para estimar el modo:

1. Haz un movimiento rápido del brazo.
2. Desde la interfaz, "Estimar modo", o `ros2 service call /adrc_simple/estimar_modo std_srvs/srv/Trigger`, o:

```bash
ros2 run control_adrc_rover estimar_modo --segundos 8
```

Se toma φ̂ (perturbación estimada) tras el movimiento, se busca el pico del espectro (0.8–20 Hz) y se estima ζ (decremento logarítmico o media potencia). Si el pico no destaca del ruido se informa "sin pico claro".

3. Aplica: `shaper.tipo zvd`, `shaper.frecuencia_hz`, `shaper.amortiguamiento` (con el brazo quieto).

**IMU (más adelante)**: pon `interfaz.topico_imu` (p. ej. `/imu_data`); el servicio `estimar_modo` también analiza su aceleración. El diseño ya separa la estimación (`nucleo/shaper.py`) del origen de la señal.

## 17. Pruebas automáticas

En la PC (sin ROS corriendo):

```bash
cd ~/ros2_ws/src/control_adrc_rover
python3 -m pytest -q test --ignore=test/test_flake8.py --ignore=test/test_pep257.py --ignore=test/test_copyright.py
cd ../control_adrc_gui && python3 -m pytest -q test
```

Incluyen: configuración y validación, GPI (rechazo de perturbación, φ̂, anti-windup, cambio de polos sin salto), trayectoria (fronteras, límites, replaneo continuo), protecciones, conformador (ZVD elimina > 95 % de la vibración) y estimación del modo, supervisor, estimador de κ, identificación, modelo reducido (ausentes, sólo J1, validación del árbol), lazo completo con una planta en Python (sin sobreimpulso, sólo J1, el caso J1+J2 con masas de más que oscila y su corrección, watchdog, paro), Monte Carlo con masas, armadura y K_tau ±50 % (`MC_N=30` para más casos) y el autotuning.

Gazebo con masas distintas: `escala_masa:=0.6` o `1.4` en `gazebo.launch.py`.

## 18. Problemas comunes

- **Nunca dice "Cero fijado"**: no llega la medición. Revisa `ros2 topic hz` del `topico_estado`; si sale "no encuentro [...] Nombres recibidos: [...]", copia esos nombres a `nombre_en_estado`; en la Jetson prueba `qos_estado_best_effort: true`; en sim revisa que corra el puente (`gazebo.launch.py` ya lo lanza).
- **"Configuración inválida: …"**: el mensaje dice el archivo, la clave y el valor. Casi siempre es un número sin punto decimal, un límite por encima del motor o una articulación activa que cuelga de una ausente.
- **Oscila**: revisa que los estados coincidan con lo montado (κ). Mira κ en la interfaz: > 1.4 → baja β o corre el autotuning; < 0.6 → el modelo subestima la inercia: autotuning (bajar wn no sirve). Revisa la frecuencia de la medición y `signo`.
- **Estado falla "sin medición"**: el watchdog quitó la corriente. Cuando vuelva la medición, "Habilitar".
- **Objetivo rechazado "sostener el brazo… pide X A"**: esa postura excede el límite de corriente; es un límite físico.
- **Muy lento / "alargada para no exceder la corriente"**: el motor no da para más en esa postura (típico: hombro con el brazo estirado).
- **La interfaz no abre**: en WSL usa la IP que imprime la terminal (`http://172.x.x.x:8080`); a veces Windows no reenvía `localhost` a WSL. Si quedó un proceso viejo ocupando el puerto, ciérralo (`pkill -f control_gui`).
- **El paro no se quita**: "Liberar paro" y después "Habilitar" (son dos pasos a propósito).
- **colcon falla en control_adrc_interfaces**: compila desde `~/ros2_ws` y verifica `ros-humble-rosidl-default-generators` (viene con ROS).

## 19. Reglas de seguridad (robot real)

1. Primero en simulación con el mismo perfil (sólo cambia la plataforma).
2. Paro de emergencia FÍSICO a la mano y fuente con límite de corriente. El paro de la interfaz es de software.
3. Primeras pruebas: sin carga, 10 % de velocidad, 5–10°, una articulación a la vez (`.nan` o "Mover sólo ésta").
4. Verifica el sentido de giro de cada motor ANTES de cerrar el lazo (`signo`).
5. Configura también el límite de corriente en el propio driver.
6. Nunca subas `limite_mando` por encima de la corriente continua (Maxon 9.34 A) ni del par continuo del reductor (27 N·m → 6.86 A con K_tau = 3.94). El control lo rechaza.
7. Con encoders incrementales, enciende el brazo SIEMPRE en la postura de `postura_encendido_urdf`.
8. Si al arrancar el brazo no está quieto, detenlo: el cero se fija con la primera medición.
9. Usa el modo calibración para autotuning y ajustes: si se cierra la página, el control se para.
10. El autotuning mueve el brazo: revisa las posturas antes.

## 20. Valores supuestos que conviene verificar

- Sinfín de J1 y J4: 10:1 con eficiencia 0.5; caja Yellow Jacket 50.9:1 (`motores.yellow_jacket_sinfin`).
- Inercia del rotor Yellow Jacket: 3e-6 kg·m² (armadura ≈ 0.777).
- Corriente segura Yellow Jacket: 2.5 A en el robot (`plataformas/real.yaml`); 6.54 A en simulación.
- Eficiencia del GSW 62 A: 0.75 (K_tau = 3.94 N·m/A).
- J5 (servo): se manda posición; se supuso que no reporta posición (`medida: false`) y que su driver espera radianes.
- Posturas del autotuning.

El autotuning identifica la inercia, la armadura y la fricción "vistas" por el lazo (ya incluyen el error de K_tau).

## 21. Glosario

- **ADRC / GPI**: control por rechazo activo de perturbaciones; el GPI (Sira-Ramírez) usa integrales del error en lugar de un observador explícito.
- **φ̂ (phi_hat)**: perturbación estimada por los integradores [rad/s²].
- **κ (kappa)**: M_ctrl/M_real; 1 es ideal.
- **β (beta)**: factor ≤ 1 de la inercia usada por el control.
- **α, α_g**: escalas de inercia y gravedad identificadas.
- **Armadura**: inercia del rotor × reducción².
- **Activa / bloqueada / ausente**: se controla / fija con masa / no montada.
- **Inversa de B**: i = K⁻¹(M v + g).
- **Anti-windup**: evita que los integradores crezcan mientras la corriente satura.
- **Watchdog**: vigila que la medición siga llegando.
- **Conformador (input shaper)**: filtro de la referencia que no excita un modo de vibración.
- **Lifecycle**: estados configurar/activar/desactivar de un nodo.
- **Heartbeat**: latido periódico que prueba que la interfaz sigue viva.
- **Latched (transient_local)**: el último valor publicado llega también a quien se suscribe después.
- 1 rad = 57.3°. ROS trabaja en radianes; tú mandas grados.
