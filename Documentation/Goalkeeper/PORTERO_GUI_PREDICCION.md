# GUI, patadas y predicción del portero

Este documento describe el panel del portero, cómo operarlo y cómo probar sus
funciones sin poner el Booster en riesgo. El panel funciona en la red local y
no depende de Internet.

## Resultado

- Panel web en `http://IP_DEL_ROBOT:8088`.
- 102 parámetros agrupados: READY/cobertura, persecución, ajuste/posesión,
  cámara, patadas, predictor y bloqueo predictivo.
- Aplicación inmediata mediante parámetros ROS 2.
- Persistencia en
  `beijing/beijing_ws/src/brain/config/config_local.yaml`.
- Selector en vivo entre Kick convencional y RLVisionKick/VisualKick.
- Predictor con historial, regresión ponderada, control de calidad, velocidad,
  desaceleración, cruce con la línea defensiva y movimiento de intercepción.
- Telemetría por topics ROS y visualización de trayectoria en Rerun.
- Restauración del perfil original desde la GUI.

La predicción está desactivada por defecto para evitar movimiento lateral por
una detección todavía no validada.

## Configuración de despliegue

La rama está preparada actualmente para:

| Ajuste | Valor |
|---|---:|
| Equipo | `5` |
| Jugador/portero | `1` |
| Rol | `goal_keeper` |
| GameController | `10.0.16.150` |
| Rerun Viewer | `10.0.16.110:9876` |

La IP `10.0.16.150` aparece en `brain/config/config.yaml` y en
`game_controller/launch/launch.py`. Deben cambiarse ambas si cambia la
computadora del GameController. Rerun sólo transmite por red cuando
`rerunLog.enable_tcp=true`.

## Inicio y acceso

```bash
cd Pumanoids/beijing
bash scripts/prepare_goalkeeper_build.sh
./scripts/start.sh role:=goal_keeper
```

Inicio independiente del panel:

```bash
cd Pumanoids/beijing
bash scripts/start_goalkeeper_web.sh
tail -f goalkeeper_web.log
```

Para cambiar el puerto:

```bash
GOALKEEPER_WEB_PORT=8090 bash scripts/start_goalkeeper_web.sh
```

`scripts/stop.sh` detiene también el panel. Éste escucha en `0.0.0.0`; debe
usarse únicamente en una red de práctica confiable porque no incorpora
autenticación ni TLS.

## Cómo aplicar cambios

1. Elevar el robot, dejarlo en `INITIAL` o despejar completamente el área.
2. Pulsar **Recargar** para leer los valores actuales de `/brain_node`.
3. Modificar el formulario. Esto todavía no envía nada al robot.
4. Pulsar **Aplicar en vivo** para probar durante el proceso actual. Los
   cambios se pierden al reiniciar.
5. Tras validar el resultado, pulsar **Aplicar y guardar**. Además de aplicarse
   inmediatamente, se escribe `config_local.yaml` de forma atómica.

Aplicar cualquiera de las dos opciones puede modificar inmediatamente el
movimiento cuando el GameController permite actuar.

El servidor comprueba tipos, límites y relaciones. Rechaza, por ejemplo, más
muestras mínimas que máximas o un tiempo mínimo de bloqueo superior al máximo.

## Restaurar el comportamiento original

El perfil `factory_defaults.json` conserva los 102 valores conocidos y no se
modifica al guardar ajustes locales.

1. Abrir **Ayuda y restauración**.
2. Pulsar **Cargar todos los valores originales**.
3. Confirmar y revisar el formulario. Aún no se ha aplicado nada.
4. Pulsar **Aplicar y guardar**.

Los valores centrales restaurados son:

```text
goalkeeper.mode=attack
goalkeeper.kick.type=default
goalkeeper.prediction.enabled=false
goalkeeper.prediction.require_localization=true
```

## Selección de patada

`goalkeeper.kick.type` se consulta en cada decisión:

- `default`: ejecuta Kick, el despeje convencional del demo original;
- `visual`: ejecuta RLVisionKick mediante la API VisualKick de la SDK.

El panel expone alineación, velocidad, duración, estabilización y salida del
Kick convencional; versión kV1/kV2, esperas y tiempos de VisualKick; además de
protecciones por obstáculos y robot caído. Los ajustes de ambos tipos se
conservan, pero sólo se ejecuta el seleccionado.

Cambiar el tipo durante una ejecución detiene la acción anterior mediante el
manejo `onHalted()` del árbol.

### Potencia y rapidez de la patada

- En `default`, `goalkeeper.kick.default.speed_limit` controla la velocidad
  límite del `crabWalk` que empuja/despeja el balón. El perfil recomendado usa
  1.5 m/s. Aumentarlo hasta 2.0 puede hacer el contacto más rápido, pero no lo
  convierte en una patada articulada y aumenta el riesgo de perder estabilidad.
- En `visual`, la API accesible no recibe un valor de potencia. `kV1/kV2` elige
  la versión de VisualKick y las esperas sólo cambian la preparación/salida.
- `strategy.power_shoot` no se habilita para el portero: en este código
  `AdjustForPowerShoot` y `Shoot` son stubs, y `useStrongShoot` está marcado
  como imposible. Activarlo daría una falsa sensación de funcionalidad.

## Funcionamiento del predictor

1. Cada detección del balón se convierte a coordenadas del campo.
2. Se descartan detecciones inferiores a `min_ball_confidence`.
3. Por defecto no se predice hasta que `odom_calibrated=true`.
4. El predictor descarta posiciones fuera del campo más `field_margin`; con
   `reject_outside_field=true` también se rechaza el candidato antes de que los
   comportamientos lo acepten. Este segundo filtro exige localización calibrada
   y una pose del robot geométricamente plausible.
5. Un salto superior a `max_sample_jump` reinicia el historial.
5. Se ajustan `x(t)` e `y(t)` por mínimos cuadrados ponderados.
6. Se calculan `vx`, `vy`, rapidez, R² y residual RMS.
7. El resultado debe cumplir muestras, intervalo, rapidez mínima/máxima y
   calidad.
8. Sólo se considera tiro si apunta a la portería propia y supera
   `min_toward_goal_speed`.
9. La desaceleración determina si alcanza la línea defensiva y cuándo.
10. Si cruza dentro de la portería y del horizonte temporal, la decisión pasa
    a `block_shot` con prioridad sobre chase, adjust y kick.
11. `GoalkeeperBlockShot` mueve el cuerpo al punto previsto usando los límites
    del grupo **Bloqueo predictivo**.
12. Si queda menos de `block.urgent_time_sec` y el error lateral supera
    `block.urgent_lateral_error`, el modo urgente limita X/giro y prioriza Y.
13. La compensación de zona muerta de `block.apply_min_velocity` se aplica sólo
    a Y. X y giro no reciben el piso global de `0.4`, porque eso sobrepasaría
    sus límites urgentes y convertiría el bloqueo en una diagonal involuntaria.

## Parámetros que más afectan la respuesta

- Menor latencia: reducir cuidadosamente `min_samples` y `min_span_msec`, o
  aumentar `recency_weight`.
- Más sensibilidad: reducir `min_speed` o `min_toward_goal_speed`; puede
  aumentar falsos positivos.
- Menos falsos positivos: aumentar `min_ball_confidence`/`min_r_squared` y
  reducir `max_residual`.
- Activación anticipada: aumentar `max_time_to_block`.
- Movimiento lateral más rápido: aumentar `block.vy_limit` y
  `block.position_gain` dentro de límites seguros.
- Bloqueo urgente lateral puro: usar `block.urgent_vx_limit=0` y
  `block.urgent_vtheta_limit=0`. Estos son los valores recomendados medidos.
- Corrección diagonal experimental hacia la línea defensiva fija: aumentar
  gradualmente `block.urgent_vx_limit`. Esto no selecciona todavía un punto
  adelantado sobre la trayectoria del balón y no debe confundirse con el avance
  involuntario que antes introducía el piso global de velocidad.
- Menos oscilación: aumentar `activation_hold_msec` o reducir la ganancia.
- Adaptación al césped: medir y ajustar `deceleration`.

## ROS y Rerun

```bash
ros2 topic echo /brain/goalkeeper/decision
ros2 topic echo /brain/goalkeeper/status
```

El estado JSON incluye decisión, patada, predictor, muestras, detección,
velocidad, R², residual, intercepción, GameController y localización.
También publica diagnóstico independiente del predictor: edad de la medición
del balón, edad/cambio de decisión, comando solicitado y enviado, velocidad de
odometría, permiso para reclamar, liderazgo, coste y tiempos
`decision_to_command_msec`, `command_to_motion_msec`,
`decision_to_motion_msec`, `command_to_aligned_motion_msec` y
`decision_to_aligned_motion_msec`.

En **Cadena de reacción**, `waiting_command` significa que Brain decidió pero
todavía no produjo movimiento; `waiting_motion` que la marcha ya recibió un
comando pero la odometría aún no confirma desplazamiento. Una alerta
`command_stopped_before_motion` identifica una orden que desapareció después
de 500 ms sin alcanzar 15 mm ni 0.02 rad.

Rerun registra:

```text
field/goalkeeper/predicted_ball_trajectory
field/goalkeeper/intercept_point
```

## Prueba segura recomendada

1. Elevar el robot y confirmar **Brain conectado**.
2. Seleccionar `default`, aplicar en vivo y verificar `kick_type` por topic.
3. Seleccionar `visual` y repetir sin balón en rango.
4. Activar el predictor elevado; mover el balón hacia la portería y comprobar
   muestras, velocidad negativa en X y trayectoria.
5. Moverlo paralelo o alejándose: no debe aparecer `block_shot`.
6. En suelo, despejar el área, disponer de paro de emergencia y comenzar con
   `goalkeeper.prediction.block.vy_limit=0.4`.
7. Validar el sentido lateral antes de aumentar gradualmente la velocidad.

No activar por primera vez durante un partido ni cerca de bordes, personas u
otros robots.

## Archivos principales

- `beijing/beijing_ws/src/brain/include/goalkeeper_ball_prediction_policy.h`
- `beijing/beijing_ws/src/brain/src/brain.cpp`
- `beijing/beijing_ws/src/brain/src/brain_tree.cpp`
- `beijing/beijing_ws/src/brain/src/robot_client.cpp`
- `beijing/beijing_ws/src/brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml`
- `beijing/beijing_ws/src/brain/tools/goalkeeper_web/`
- `beijing/scripts/start_goalkeeper_web.sh`
- `beijing/scripts/prepare_goalkeeper_build.sh`

El árbol anterior permanece en `subtree_goal_keeper_play_original.xml`.

## Compilación manual

```bash
cd Pumanoids/beijing/beijing_ws
source /opt/ros/kilted/setup.bash  # o Humble, según la imagen
colcon build --symlink-install --packages-select brain
source install/setup.bash
colcon test --packages-select brain
colcon test-result --verbose
```

Prueba web sin ROS:

```bash
python3 src/brain/test/goalkeeper_web_test.py \
  --server-dir src/brain/tools/goalkeeper_web
```
