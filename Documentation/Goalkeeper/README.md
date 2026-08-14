# Portero configurable — guía de operación

## Alcance

La rama `test_goalkeeper` se basa en `beijing_demo` y modifica únicamente el
demo ubicado en `beijing/`. El panel web controla parámetros reales del nodo
`/brain_node`; no es una simulación cuando se ejecuta en el robot.

## Flujo correcto para cambiar parámetros

1. Colocar el robot elevado, en `INITIAL` o en una zona despejada.
2. Abrir `http://IP_DEL_ROBOT:8088` y confirmar **Brain conectado**.
3. Pulsar **Recargar** para leer la configuración activa.
4. Modificar los parámetros deseados. Editar el formulario no envía cambios.
5. Pulsar **Aplicar en vivo** para una prueba temporal. Se pierde al reiniciar.
6. Tras validar el resultado, pulsar **Aplicar y guardar** para persistirlo en
   `beijing/beijing_ws/src/brain/config/config_local.yaml`.

Aplicar cualquiera de las dos opciones sí puede cambiar inmediatamente el
movimiento si el GameController permite actuar.

## Identidad y red configuradas

El portero está configurado como equipo `5`, jugador `1`, rol `goal_keeper`.
El GameController esperado es `10.0.16.150` y el visor Rerun esperado es
`10.0.16.110:9876`.

Archivos relevantes:

- `beijing/beijing_ws/src/brain/config/config.yaml`: `team_id`, `player_id`,
  `player_role`, `game_control_ip` y `rerunLog.server_ip`;
- `beijing/beijing_ws/src/game_controller/launch/launch.py`:
  `ip_white_list` del receptor UDP.

`game_control_ip` y `ip_white_list` deben apuntar al mismo GameController. La
lista blanca queda activada por defecto; si la IP no coincide, el nodo puede
estar ejecutándose correctamente pero ignorará los paquetes recibidos.

Para diagnosticar:

```bash
ros2 node list | grep game_controller
ros2 topic list | grep game
ros2 topic echo /brain/goalkeeper/status
```

El estado publicado por el portero incluye el estado de juego que recibió el
brain, lo que permite diferenciar un problema de red de uno del árbol.

## Volver al comportamiento original

El archivo `factory_defaults.json` conserva un perfil protegido de 94 valores.
No se sobrescribe cuando la GUI guarda `config_local.yaml`.

1. Abrir **Ayuda y restauración**.
2. Pulsar **Cargar todos los valores originales** y confirmar.
3. Revisar el formulario; aún no se ha aplicado nada.
4. Pulsar **Aplicar y guardar**.

Los valores esenciales del demo original son:

```text
goalkeeper.mode=attack
goalkeeper.kick.type=default
goalkeeper.prediction.enabled=false
goalkeeper.prediction.require_localization=true
```

## Modos del portero

- `attack`: cubre la portería, pero puede perseguir, ajustar y despejar el
  balón cuando `GoalieDecide` determina que debe reclamarlo.
- `guard`: prioriza permanecer en cobertura. No ejecuta la secuencia normal de
  persecución y patada.

El modo original es `attack`.

## Selección de patada

En el grupo **Patada**, cambiar `goalkeeper.kick.type`:

- `default`: usa `Kick`, el despeje convencional del demo original.
- `visual`: usa `RLVisionKick` y permite elegir `kV1` o `kV2`.

Los parámetros de ambas patadas permanecen guardados. Sólo se ejecuta el tipo
seleccionado. Probar el cambio elevado y sin balón en rango antes de una prueba
en suelo.

## Predictor de tiros

El predictor se activa con `goalkeeper.prediction.enabled=true`. Acumula
detecciones del balón, ajusta una trayectoria ponderada, calcula velocidad,
calidad, desaceleración y cruce con la línea defensiva. Si el cruce amenaza la
portería dentro del horizonte configurado, `GoalieDecide` emite `block_shot` y
el robot se desplaza al punto lateral previsto.

Mantener `goalkeeper.prediction.require_localization=true`: sin
`odom_calibrated`, el historial se limpia y no se ordena el bloqueo.

Prueba inicial recomendada:

1. Robot elevado; predictor activo y localización lista.
2. Mover el balón hacia la portería y verificar aumento de muestras.
3. Confirmar `prediction_valid`, velocidad negativa en X y punto de cruce.
4. Mover el balón alejándose: no debe aparecer `block_shot`.
5. En suelo, comenzar con `goalkeeper.prediction.block.vy_limit=0.4` y una
   persona junto al paro de emergencia.

### Tiempo de reacción

La latencia de detección se aproxima a:

```text
máximo(tiempo para reunir min_samples, min_span_msec)
+ edad de la última observación
+ un ciclo de Brain/árbol
```

El perfil rápido usa `min_samples=5`, `min_span_msec=100`,
`max_samples=20` e `history_msec=600`. La web muestra `Intervalo observado`,
`Edad de observación` y `Latencia estimada`; por eso ya no es necesario inferir
la demora a partir de ceros.

Una vez aceptado el tiro, la rapidez física depende de
`prediction.block.position_gain`, `prediction.block.vy_limit` y de la capacidad
de marcha. `reaction_margin_sec` no retrasa la detección: reserva tiempo y hace
que el comando lateral llegue antes al límite cuando el cruce es inminente.
`activation_hold_msec` evita perder el bloqueo por una detección intermitente.

La demora general del portero se mide aparte del predictor. La sección
**Cadena de reacción** correlaciona:

```text
última medición del balón -> cambio de decisión -> comando de velocidad
-> desplazamiento confirmado por odometría
```

`Decisión -> comando` identifica espera dentro de Brain/árbol;
`Comando -> movimiento` mide el arranque físico de la marcha del T2, usando
como confirmación 15 mm de traslación o 0.02 rad de giro. También se muestran
el comando solicitado, el realmente enviado después de límites/protecciones,
la velocidad de odometría, liderazgo, coste y permiso para reclamar el balón.
Estas métricas funcionan con `prediction.enabled=false`.

La rama de ataque es reactiva: `block_shot` puede interrumpir `chase`, `adjust`
o `kick`. Al terminar la amenaza, `post_block_claim_msec` mantiene durante un
intervalo corto el derecho del portero a reclamar un balón cercano; el flujo
continúa automáticamente por `chase -> adjust -> kick`.

### Visualización 2D

La GUI dibuja el campo, portería propia, línea defensiva, pose/orientación del
robot, balón, historial de observaciones, trayectoria futura y punto de
intercepción. Amarillo es historial observado, naranja es predicción y rojo es
una amenaza. También muestra R² longitudinal y lateral por separado.

## Observabilidad

El servidor web conserva cada mensaje de estado del portero (10 Hz) y cada
cambio de parámetros aplicado desde la GUI en un archivo JSON Lines. Cada línea
incluye fecha UTC, decisión, razón de aceptación
o rechazo del predictor, detección y confianza del balón, pose del robot,
velocidad estimada, R², residual, muestras y punto/tiempo de intercepción.
Además registra edades de percepción/decisión, etapa de reacción, comandos
solicitados/enviados, velocidad de odometría, permiso de reclamación y las
latencias decisión-comando-movimiento.

Etapas de reacción:

- `waiting_command`: el árbol cambió de decisión pero aún no emitió marcha;
- `waiting_motion`: el comando salió y se espera desplazamiento físico;
- `moving`: odometría confirmó la respuesta;
- `command_stopped_before_motion`: el comando terminó sin movimiento medible;
- `stopped`: hubo movimiento y posteriormente terminó.

La GUI muestra los últimos 40 eventos y permite descargar el registro. En el
robot los archivos se guardan en:

```text
~/Pumanoids/beijing/goalkeeper_logs/goalkeeper_telemetry_YYYYMMDD_HHMMSS.jsonl
```

Las razones diagnósticas incluyen `localization_required`,
`insufficient_samples`, `insufficient_span`, `speed_below_minimum`,
`r_squared_below_minimum`, `residual_above_maximum`,
`not_toward_own_goal`, `outside_goal` y `threat_detected`.

```bash
ros2 topic echo /brain/goalkeeper/decision
ros2 topic echo /brain/goalkeeper/status
```

Rerun recibe:

```text
field/goalkeeper/predicted_ball_trajectory
field/goalkeeper/intercept_point
```

Para transmitir estas entidades a la computadora configurada:

```yaml
rerunLog:
  enable_tcp: true
  server_ip: "10.0.16.110:9876"
```

Con `enable_tcp=false`, el brain no transmite por red aunque `server_ip` esté
correctamente establecido; el registro local continúa según `enable_file`.

El panel incluye un apéndice generado desde el esquema con los 94 parámetros,
valor original, rango, descripción y parte del robot afectada.

## Compilación y arranque

```bash
cd beijing
bash scripts/prepare_goalkeeper_build.sh
./scripts/start.sh role:=goal_keeper
```

El arranque operativo conserva el flujo original de GameController. La web y
el registro se levantan automáticamente desde `start.sh`; no existe un modo
alternativo que omita los estados de partido.

Consultar también [IMPLEMENTATION.md](./IMPLEMENTATION.md) antes de transferir
la rama al robot.

Documentación extendida conservada del desarrollo inicial:

- [Manual completo de GUI, patadas y predictor](./PORTERO_GUI_PREDICCION.md)
- [Manifiesto detallado de implementación](./IMPLEMENTACION_PORTERO_MANIFIESTO.md)
