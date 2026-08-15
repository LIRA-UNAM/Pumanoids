# Configuración y extensión del portero

## Orden de carga y precedencia

Brain recibe parámetros ROS 2 en este orden:

```text
config/config.yaml
        ↓ sobrescrito por
config/config_local.yaml
        ↓ sobrescrito por
argumentos construidos por launch.py
```

`config.yaml` es el perfil base versionado. `config_local.yaml` es el lugar
destinado a ajustes persistentes de la unidad. Los argumentos de launch solo
cubren opciones concretas como árbol, rol, posición, simulación y desactivación
de logs/comunicación.

Ejemplos:

```bash
ros2 launch brain launch.py tree:=game.xml role:=goal_keeper
ros2 launch brain launch.py pos:=left disable_log:=true
ros2 launch brain launch.py disable_com:=true
```

Un parámetro que aparece correctamente en el YAML fuente puede no ser el valor
activo si `config_local.yaml` o launch lo reemplaza. Compruébelo en el proceso:

```bash
ros2 param get /brain_node game.player_role
ros2 param get /brain_node goalkeeper.mode
ros2 param get /brain_node goalkeeper.kick.type
ros2 param get /brain_node goalkeeper.prediction.enabled
```

## Demo original frente al perfil de esta rama

No confunda los valores que venían en el release extraído con los elegidos para
el portero de laboratorio:

| Ajuste | Demo original auditado | Rama `test_goalkeeper` |
|---|---:|---:|
| Equipo | `29` | `5` |
| Jugador | `2` | `1` |
| Rol | `striker` | `goal_keeper` |
| GameController | `172.168.208.2` | `10.0.16.150` |
| Rerun Viewer | `192.168.10.110:9876` | `10.0.16.110:9876` |
| Rerun TCP | desactivado | desactivado |
| Rerun a archivo | activado | activado |

Estos números no son “valores de fábrica” del Booster T2; son perfiles de dos
entornos de juego. El perfil de restauración de la GUI restaura el
**comportamiento del portero** (93 parámetros), no cambia identidad, red,
calibración de cámara ni geometría del campo.

## Identidad y campo

| Parámetro | Función | Precaución |
|---|---|---|
| `game.team_id` | Número del equipo RoboCup. | Debe aparecer en uno de los dos equipos GC. |
| `game.player_id` | Índice del jugador, de 1 a 5 en este perfil. | Debe ser único; se usa como `player_id-1` para penalización. |
| `game.field_type` | Geometría `adult_size`, `kid_size` o `robo_league`. | Un mapa incorrecto corrompe localización y límites. |
| `game.player_role` | `striker` o `goal_keeper`. | Determina subárbol en PLAY. |
| `game.initial_goalkeeper_id` | Portero inicial/fallback. | `0` significa no asignado en shootout. |
| `game.player_start_pos` | Lado de entrada `left`/`right`. | Afecta restricciones de localización/READY. |
| `game.number_of_players` | Tamaño lógico del equipo. | No inventa compañeros ausentes. |
| `game.treat_person_as_robot` | Trata Person como robot para depuración. | Mantener `false` en partido oficial. |

`game_control_ip` identifica la PC de GameController para la comunicación de
retorno. El receptor UDP tiene además `ip_white_list` en su launch. Cambie los
dos cuando cambie la computadora del árbitro.

## Parámetros generales del T2

### Robot y marcha

| Parámetro/grupo | Qué mueve o corrige |
|---|---|
| `robot.robot_height` | Altura usada por proyección/cinemática; `1.28146` en el perfil T2. |
| `robot.odom_factor` | Escala de traslación de `/odometer_state`. |
| `robot.odom_theta_offset` | Offset angular manual entre odometría y marco esperado. |
| `robot.odom_theta_auto_align` | Estima automáticamente ese offset durante traslación sostenida. |
| `robot.odom_theta_alignment_*` | Distancia y concentración exigidas antes de fijar alineación. |
| `robot.vx_factor` | Ganancia sobre velocidad frontal. |
| `robot.yaw_offset` | Corrección angular usada por marcha/patada. |
| `robot.vx_limit`, `vy_limit`, `vtheta_limit` | Límites globales del comando. |
| `robot.min_vx`, `min_vy`, `min_vtheta` | Compensación de zona muerta para comandos no nulos. |

Subir mínimos puede convertir una corrección pequeña en un paso brusco. Ajuste
primero escalas/offsets, después mínimos y al final límites máximos.

### Estrategia

`strategy.*` controla memoria y confianza del balón, búsqueda distribuida,
velocidad cerca del balón/borde, patada, shoot/power shoot, VisualKick
automático, tiros libres y cooperación.

Grupos más importantes:

- `ball_confidence_threshold`, `ball_confidence_decay_rate` y
  `ball_memory_timeout`: cuándo el balón sigue considerándose conocido;
- `search.*`: giro inicial, tiempo por waypoint, timeout, velocidades y
  tolerancia de llegada;
- `kick_range`, `kick_theta_range` y abortos por balón movido: transición de
  ajuste a patada;
- `enable_auto_visual_kick` y límites de distancia/ángulo: ruta automática del
  delantero, no el selector particular del portero;
- `freekick.*`: selección y timeout del plan colectivo;
- `cooperation.*`: elección de líder, claim del portero, kickoff, formación,
  asistencia y evasión entre compañeros.

La ausencia de otros robots no impide localización, pero hace que los timeouts y
fallbacks de cooperación sean relevantes.

### Evitación de obstáculos

`obstacle_avoidance.*` cubre cuatro capas:

| Grupo | Parámetros representativos |
|---|---|
| Profundidad/grid | `depth_sample_step`, confirm/clear frames, altura, grid, ocupación. |
| Geometría segura | radio propio, distancia segura, exclusión del balón, collision threshold. |
| Robots/personas | confianza, tracking, fusión RGB-depth, radios upright/fallen, memoria. |
| Planificador | velocidades, clearance, bloqueo de lado, switch penalty y recuperación. |

`depth_sample_step` es un salto en píxeles, no una cantidad de muestras; para
544×448 el valor actual es 8. `avoid_person=true` aporta seguridad sin cambiar
la semántica de rival. Aunque los estimadores de `vision_node` tienen profundidad
desactivada, brain se suscribe directamente al topic depth para este grid; hay
que verificar por separado su frecuencia, encoding, intrínsecos y sincronía con
la pose de cabeza.

Los flags finales deciden en qué acciones se aplica la evasión:

```text
avoid_during_kick
avoid_during_chase
enable_fallen_robot_visual_kick_exit
enable_freekick_avoid
enable_ball_obstacle
```

### Localización, comunicación y observabilidad

| Grupo | Parámetros |
|---|---|
| `locator` | `min_marker_count`, `max_residual`. |
| `enable_com` | Habilita comunicación entre compañeros. |
| `communication` | Frecuencia de broadcast, máximo 20 Hz en el perfil. |
| `rerunLog` | TCP, servidor, archivo, rotación y throttling. |
| `odomLog` | Log independiente de odometría, frecuencia y flush. |
| `debug` | Logs de desarrollo, por ejemplo posición del balón. |
| `sound` | Sonido y paquete utilizado. |

### Recuperación de caída

`recovery.*` define una máquina de estados segura:

```text
detectar caída
→ solicitar/confirmar Prepare
→ GetUp V1 o V2
→ confirmar postura erguida y quieta
→ solicitar/confirmar Soccer
→ esperar estabilización de odometría
→ realinear heading con visión
→ reanudar localización y árbol
```

Los timeouts no deben reducirse solo para “responder más rápido”. Una liberación
antes de que el soporte y la odometría se estabilicen puede desplazar toda la
pose del campo.

## Dos predictores distintos

El YAML contiene dos grupos con nombres parecidos:

- `ball_predictor.*`: predictor general heredado, con steps, R² y aceleración;
- `goalkeeper.prediction.*`: predictor de disparos implementado para esta rama,
  integrado con `GoalieDecide`, GUI, topics y Rerun.

Para la función de bloqueo lateral del portero ajuste el segundo. Cambiar solo
`ball_predictor` no activa `block_shot`.

## Modos del portero

| Modo | Comportamiento |
|---|---|
| `attack` | Cubre la portería y puede reclamar, perseguir, ajustar y despejar. |
| `guard` | Prioriza cobertura y bloqueo; evita la secuencia ofensiva normal. |

El valor original es `attack`.

### READY y cobertura

- `ready.dist_to_goalline`: profundidad objetivo desde la línea de gol;
- `ready.dist_tolerance`/`theta_tolerance`: cuándo se considera colocado;
- `ready.long_range_threshold`/`turn_threshold`: estrategia de aproximación;
- `ready.vx_limit`, `vy_limit`, `vtheta_limit`: máximos de colocación;
- `ready.avoid_obstacles`: activa planificador en READY;
- `blocking.dist_to_goalline`: línea defensiva usada durante cobertura;
- `blocking.position_gain` y `orientation_gain`: respuesta a error;
- `blocking.*_limit`: límites del movimiento de cobertura normal.

Reducir tolerancias mejora precisión aparente, pero puede provocar correcciones
continuas. Subir ganancias/velocidades reduce tiempo de llegada y aumenta
oscilación, deslizamiento y riesgo lateral.

### Persecución, ajuste y claim

| Grupo | Efecto |
|---|---|
| `chase.threshold` | Distancia/criterio de `GoalieDecide` para perseguir. |
| `chase.target_distance` | Separación objetivo respecto al balón. |
| `chase.safe_distance` | Clearance para evasión. |
| `chase.*_limit` | Velocidades durante persecución. |
| `adjust.turn_threshold`, `range` | Umbrales para alinear la patada. |
| `adjust.*_limit` | Velocidades finas alrededor del balón. |
| `claim.max_ball_range` | Solo reclama balones cercanos observados localmente. |
| `claim.max_cost` | Coste máximo para asumir control. |
| `claim.extra_depth`, `lateral_margin` | Región adicional alrededor del área. |
| `claim.require_team_lead` | Exige autoridad colectiva antes de salir. |

Un portero lento frente a un tiro lateral puede estar limitado antes por una
predicción inválida o decisión `retreat`, no por `vy_limit`. Mire la decisión y
el status antes de aumentar velocidad.

### Cabeza del portero

`goalkeeper.camera.*` controla seguimiento/búsqueda:

- tolerancia de centrado y factor central;
- constante de tiempo del filtro;
- intervalo mínimo entre comandos;
- velocidad pitch/yaw de tracking;
- cambio mínimo que merece un RPC;
- duración del ciclo y velocidades de búsqueda.

Para menor latencia se puede reducir moderadamente el filtro/intervalo o subir
las tasas. Valores agresivos crean jitter, saturan RPC y empeoran la proyección
si imagen y pose de cabeza dejan de estar sincronizadas.

## Patada convencional y VisualKick

Selector:

```yaml
goalkeeper:
  kick:
    type: "default"   # default | visual
```

### `default` → nodo `Kick`

El nodo convencional alinea, opcionalmente estabiliza y publica la referencia
de patada. Parámetros:

- `alignment_tolerance`;
- `default.speed_limit`;
- `default.min_msec`;
- `default.enable_stabilize` y `stabilize_msec`;
- `default.exit_range`;
- aborto y umbral cuando el balón se mueve.

Es el comportamiento original del portero.

### `visual` → nodo `RLVisionKick`

Invoca `VisualKick` de la SDK:

- `RLVisionKick.visual_kick_version`: `kV1` o `kV2`;
- `visual.min_msec`/`max_msec`;
- `visual.range`;
- `visual.pre_delay_msec`/`post_delay_msec`.

El `BodyControl::kInsideFoot` corresponde históricamente a VisualKick V2; V1
tiene su propio body control. Cambiar el tipo durante ejecución provoca halt de
la acción anterior en esta rama, pero la prueba inicial debe hacerse elevada y
sin balón en rango.

## Predictor de disparos del portero

Se activa con:

```yaml
goalkeeper:
  prediction:
    enabled: true
    require_localization: true
```

Mantenga `require_localization=true`. Sin pose de campo confiable no existe una
línea de gol absoluta segura hacia la que extrapolar.

### Flujo

1. convierte cada detección válida a campo;
2. rechaza confianza baja y saltos grandes;
3. mantiene una ventana temporal limitada;
4. ajusta `x(t)` e `y(t)` con mayor peso a muestras recientes;
5. calcula `vx`, `vy`, rapidez, R² y residual;
6. aplica desaceleración para estimar si alcanzará la línea defensiva;
7. valida dirección hacia la portería, horizonte y ancho más margen;
8. emite `block_shot` con prioridad sobre chase/adjust/kick;
9. `GoalkeeperBlockShot` manda desplazamiento al punto lateral previsto.

### Calidad y sensibilidad

| Parámetro | Si se reduce | Si se aumenta |
|---|---|---|
| `min_samples` | Respuesta más rápida, más falsos tiros. | Más robustez, más latencia. |
| `min_span_msec` | Acepta trayectoria antes. | Mide velocidad con más historia. |
| `min_speed` | Detecta movimientos lentos/ruido. | Ignora tiros lentos. |
| `min_toward_goal_speed` | Más sensible a componente frontal pequeña. | Exige tiro claramente entrante. |
| `min_r_squared` | Tolera trayectorias irregulares. | Exige linealidad. |
| `max_residual` | Exige ajuste más limpio. | Tolera dispersión. |
| `recency_weight` | Historia más uniforme. | Reacciona más a lo reciente. |
| `deceleration` | Predice mayor alcance. | Predice frenado anterior. |
| `max_sample_jump` | Reinicia con saltos menores. | Puede conservar outliers. |
| `activation_hold_msec` | Sale rápido de bloqueo. | Evita parpadeo de decisión. |

`history_msec`, `max_samples` y `step_interval_msec` limitan memoria y curva
visualizada. `goal_margin` amplía el ancho considerado peligroso.

### Movimiento de bloqueo predictivo

- `block.vx_limit`, `vy_limit`, `vtheta_limit`: máximos;
- `block.position_gain`: respuesta al error lateral;
- `block.reaction_margin_sec`: anticipación temporal;
- `block.target_tolerance`: zona muerta alrededor del objetivo;
- `block.apply_min_velocity`: compensa la zona muerta sólo en el eje lateral
  Y. No eleva X ni giro, para respetar los límites del bloqueo urgente.

El ajuste seguro es: validar primero el signo lateral con el robot elevado,
después probar en suelo con `vy_limit=0.4`, y subir gradualmente sin tocar a la
vez ganancia, tolerancia y velocidad.

## GUI web

La extensión sirve el panel en:

```text
http://IP_DEL_ROBOT:8088
```

Arranque independiente:

```bash
cd ~/T2_5v5Demo_Whrg
bash scripts/start_goalkeeper_web.sh
tail -F goalkeeper_web.log
```

El servidor escucha en `0.0.0.0`, sin autenticación ni TLS. Úselo solo en una
red de práctica confiable; no exponga el puerto 8088 a Internet.

### Flujo correcto para cambiar algo

1. ponga el robot en soporte, `INITIAL` o una zona despejada;
2. abra la GUI y confirme **Brain conectado**;
3. pulse **Recargar** para leer `/brain_node`;
4. modifique el formulario; editar no aplica nada todavía;
5. pulse **Aplicar en vivo** para una prueba temporal;
6. compruebe topics, logs y movimiento;
7. pulse **Aplicar y guardar** solo después de validar.

“Aplicar en vivo” usa parámetros ROS y se pierde al reiniciar. “Aplicar y
guardar” también escribe de forma atómica en `config_local.yaml`. Ambos pueden
cambiar inmediatamente el movimiento si el estado de partido permite actuar.

La GUI valida tipos, rangos y relaciones, por ejemplo muestras mínimas no
mayores que máximas y tiempo mínimo de bloqueo no mayor que máximo.

### Restaurar el comportamiento original

`tools/goalkeeper_web/factory_defaults.json` conserva el perfil protegido de 94
parámetros. No se sobrescribe al guardar ajustes locales.

1. abra **Ayuda y restauración**;
2. pulse **Cargar todos los valores originales**;
3. confirme y revise el formulario;
4. pulse **Aplicar y guardar**.

Valores esenciales restaurados:

```text
goalkeeper.mode=attack
goalkeeper.kick.type=default
goalkeeper.prediction.enabled=false
goalkeeper.prediction.require_localization=true
```

La carga de valores originales solo rellena el formulario hasta que se aplica.

## Observación del portero

```bash
ros2 topic echo /brain/goalkeeper/decision
ros2 topic echo /brain/goalkeeper/status
```

El status JSON incluye decisión, kick, flags del predictor, localización,
calidad, muestras, velocidad, cruce y estado GameController. La sección
**Cadena de reacción** añade comandos solicitado/enviado, velocidad física por
odometría, permiso de reclamación y latencias decisión-comando-movimiento; no
depende de que el predictor esté activo. Se publica con QoS
reliable/transient-local, por lo que un visor nuevo recibe el estado reciente.

Rerun:

```text
field/goalkeeper/predicted_ball_trajectory
field/goalkeeper/intercept_point
```

## Documentación específica relacionada

- [Guía de operación del portero](../Goalkeeper/README.md)
- [GUI, patadas y predicción](../Goalkeeper/PORTERO_GUI_PREDICCION.md)
- [Manifiesto de implementación](../Goalkeeper/IMPLEMENTACION_PORTERO_MANIFIESTO.md)
- [Detalle técnico de implementación](../Goalkeeper/IMPLEMENTATION.md)
