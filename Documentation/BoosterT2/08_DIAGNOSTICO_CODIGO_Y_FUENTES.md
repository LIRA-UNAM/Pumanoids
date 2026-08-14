# Diagnóstico, pruebas, mapa del código y fuentes

## Diagnóstico por capas

No cambie parámetros de movimiento hasta saber en qué capa se perdió el flujo.

```text
1. proceso vivo
2. red/DDS/UDP
3. topic con frecuencia y datos válidos
4. configuración/identidad correcta
5. estado/condición del árbol
6. petición locomotora enviada
7. respuesta y estado del SDK
8. movimiento físico coherente
```

## Foto inicial del sistema

```bash
date
uname -a
cat /etc/os-release
ip -br address
ip route
ps -ef | grep -E 'brain_node|vision_node|game_controller_node' | grep -v grep
ros2 node list
ros2 topic list | sort
for pkg in brain vision game_controller; do ros2 pkg prefix "$pkg"; done
```

Guarde esta salida junto con los logs. Si `ros2 pkg prefix` apunta a otro
workspace, cualquier análisis del código actual será engañoso.

## Matriz rápida de fallos

| Síntoma | Pruebas primero | Causas frecuentes |
|---|---|---|
| Los dos robots inmóviles | GC topic, stopped/penalty/state, IP, equipo, árbol | Paquetes UDP no recibidos, fase SET/STOP, equipo incorrecto, GC incompatible. |
| Solo uno inmóvil | player ID, penalización individual, mando, visión/localización | ID duplicado/fuera de rango, stick con deriva, rol/config diferente, nodo caído. |
| No mueve cabeza en INITIAL | árbol, `Calibrated`, head topic y Loco RPC | Árbol equivocado, ya calibrado y siguiendo balón, control state/manual, pose/cámara ausente. |
| Solo responde tras `LT+B` | control state y `go_manual` | No se cargó `game.xml`, stick >0.1, LT mal mapeado, blackboard sobrescrito. |
| No localiza | líneas, marcas, pose de cabeza, field type | Menos de 4 marcas, residual >0.4, mala extrínseca, mapa incorrecto, timestamps. |
| Rerun vacío | `.rrd`, flags, conexión 9876 | logging desactivado, Viewer incompatible, firewall/ruta, panel sin entidades. |
| Portero no bloquea lateral | status del predictor y decisión | predictor off, localización requerida, velocidad/dirección/calidad inválida, cruce fuera del gol. |
| Portero sale tarde o no inicia marcha | `reaction_stage`, comando enviado, odometría, permiso de reclamación | umbral de reclamación 1.5 m, no es líder, coste/zona inválidos, árbol no envió comando o arranque físico tardío. |
| Patada visual no inicia | kick type, estado SDK, protección | `default` activo, VisualKick no disponible, robot caído/obstáculo, rango/timing. |

## GameController

En el robot:

```bash
ss -lunp | grep ':3838'
tail -F game_controller.log
ros2 topic hz /robocup/game_controller
ros2 topic echo /robocup/game_controller --once
```

Si no llegan datagramas:

```bash
sudo tcpdump -ni any udp port 3838
```

Interpretación del log de esta rama:

- `no datagram received`: red, broadcast, firewall o IP/interfaz;
- `not in ... allowlist`: corrija `ip_white_list`;
- `invalid length/header/version`: GameController no usa el protocolo esperado;
- `contains invalid field values`: el paquete no pasa validación;
- `datagrams=N accepted=0`: llegan paquetes, pero todos se rechazan;
- `accepted` crece: el receptor publica; siga hacia brain.

Compruebe que `game.team_id` aparece en el mensaje y que `player_id` está dentro
de `players_per_team`. Mire `stopped`, estado, set play, kicking team y la
penalización del jugador.

## Brain y árbol

```bash
tail -F brain.log
ros2 node info /brain_node
ros2 param get /brain_node tree_file_path
ros2 param get /brain_node game.team_id
ros2 param get /brain_node game.player_id
ros2 param get /brain_node game.player_role
ros2 param dump /brain_node > /tmp/brain-active-params.yaml
```

El log periódico contiene pose/calibración, estado GC, comunicación, logging,
`vxFactor`, `yawOffset`, `ControlState` y tiempo de tick.

No existe un topic genérico del nodo activo del Behavior Tree en el demo
original. Para el portero de esta rama:

```bash
ros2 topic echo /brain/goalkeeper/decision
ros2 topic echo /brain/goalkeeper/status
```

Para separar el retardo sin depender del predictor, observe en el status:

```text
ball_measurement_age_msec
decision_input_age_msec
reaction_stage
decision_to_command_msec
command_to_motion_msec
decision_to_motion_msec
command_requested_*
command_sent_*
odom_velocity_* / odom_speed
goalkeeper_may_claim / team_lead / claim_cost
```

`waiting_command` apunta a lógica/árbol; `waiting_motion` apunta a la cadena de
locomoción o al arranque físico. `command_stopped_before_motion` significa que
Brain dejó de ordenar marcha sin que la odometría confirmara 15 mm o 0.02 rad.

Para el árbol general, añada logging de transición de BehaviorTree.CPP o
publique explícitamente el blackboard si necesita telemetría; no confunda el
topic de decisión del portero con un inspector completo del árbol.

## Mando

```bash
ros2 topic info -v /remote_controller_state
ros2 topic echo /remote_controller_state
```

Pruebe cada botón por separado y verifique nombres. En reposo, `lx`, `ly`, `rx`
y `ry` deben permanecer por debajo de 0.1 en magnitud. Un mando que publica
`/joy` no controla brain sin puente al mensaje Booster.

## Cámara y visión

```bash
ros2 topic hz /boostercamera/head/rgb
ros2 topic hz /boostercamera/head/depth
ros2 topic echo /boostercamera/head/rgb/camera_info --once
ros2 topic list | grep head_pose
ros2 topic hz /head_pose_stamped
ros2 topic hz /booster_vision/detection
ros2 topic hz /booster_vision/line_segments  # no existirá con segmentación desactivada
tail -F vision.log
```

Si hay RGB pero no detecciones, revise TensorRT/engine y el log. Si hay cajas
pero no coordenadas útiles, revise intrínsecos, extrínsecos, profundidad y
sincronía de cabeza. En el perfil actual la segmentación y el uso de profundidad
dentro de los estimadores de visión están desactivados deliberadamente; brain
aún procesa directamente el depth para obstáculos. La localización puede usar
las detecciones L/T/X/P del detector; no espere `line_segments` en ese perfil.

## Localización

```bash
grep -n 'FIELD_POSE\|Calibrated\|locat' brain.log | tail -n 100
```

Clasifique el fallo:

- código 4: muestre más marcas o baje el mínimo solo para diagnóstico;
- código 3: hipótesis no converge; revise orientación inicial/restricciones;
- código 2: ajuste incompatible; revise mapa, extrínsecos y outliers;
- pose estable pero incorrecta: simetría del campo o heading inicial;
- pose deriva: odometría/escala/offset o falta de correcciones visuales;
- ninguna cabeza: condición de árbol o RPC, no el filtro de partículas.

No “arregle” una cámara descalibrada subiendo `max_residual`: solo aceptará
poses peores.

## SDK y bridge locomotor

```bash
ros2 topic info -v /LocoApiTopicReq
ros2 topic info -v /LocoApiTopicResp
ros2 topic echo /LocoApiTopicResp
ros2 topic hz /odometer_state
ros2 topic hz /low_state
```

Si hay peticiones pero ninguna respuesta, revise bridge Booster, perfil Fast
DDS, dominio/interfaz y servicio `loco`. Si la respuesta es éxito pero no hay
movimiento, consulte `GetStatus`, modo/body control y condiciones de seguridad.

No publique manualmente en `LocoApiTopicReq` mientras brain está activo, salvo
una prueba diseñada con exclusión mutua. El script `test_visual_kick.py` es una
herramienta directa/legada: su comentario menciona Humble y su JSON simplifica
la API. Para la unidad Kilted use preferentemente la GUI, el nodo del árbol o
`B1LocoClient::VisualKick` del mismo release de SDK.

## Portero y predictor

```bash
ros2 topic echo /brain/goalkeeper/status
```

Campos útiles:

```text
decision, kick_type
prediction_enabled
localization_ready / localization_required
prediction_valid / threatens_goal
ball_detected / ball_range
velocity_x / velocity_y / speed
r_squared / residual / sample_count
intercept_x / intercept_y / time_to_intercept
game_state
```

Secuencia diagnóstica para un tiro lateral:

1. `ball_detected=true`;
2. `sample_count` llega al mínimo;
3. `prediction_valid=true`;
4. la componente X apunta a la portería propia;
5. `threatens_goal=true`;
6. `decision=block_shot`;
7. el punto de intercepción está dentro de límites;
8. aparece comando lateral y el T2 responde.

El primer paso que falle identifica el grupo de parámetros relevante. No suba
`block.vy_limit` si nunca se produce `block_shot`.

## Rerun

```bash
find /home/booster/Workspace/rrlog -type f -name '*.rrd' -ls
grep -n 'rerun\|Rerun\|Connect rerun' brain.log | tail -n 50
nc -vz IP_PC 9876
```

Pruebe primero archivo, luego TCP. Si el archivo contiene `field/mapLines` pero
no `field/robot`, brain inició logging pero no actualiza pose. Si no contiene
ninguna entidad, el problema es configuración/ruta/proceso.

## Compilación y pruebas

En el workspace desplegado:

```bash
cd ~/T2_5v5Demo_Whrg
source /opt/ros/kilted/setup.bash
colcon build --symlink-install --packages-select brain
source install/setup.bash
colcon test --packages-select brain
colcon test-result --verbose
```

Prueba web sin arrancar el robot:

```bash
python3 src/brain/test/goalkeeper_web_test.py \
  --server-dir src/brain/tools/goalkeeper_web
```

La rama incluye tests para predictor del portero, GUI, escaneo de cabeza,
búsqueda de balón, recuperación, obstáculos, planificador, tiros libres,
asistencia y compatibilidad SDK. Los tests de visión cubren sincronizador,
intrínsecos, color y modelos.

El script de preflight del repositorio ejecuta validación estática, test web,
compilación y tests de brain:

```bash
cd beijing
bash scripts/prepare_goalkeeper_build.sh
```

## Dos layouts que no deben confundirse

El release del robot es plano:

```text
T2_5v5Demo_Whrg/src
T2_5v5Demo_Whrg/scripts
T2_5v5Demo_Whrg/install
```

En esta rama los fuentes están almacenados para desarrollo en:

```text
beijing/beijing_ws/src
```

y los scripts están en `beijing/scripts`. `prepare_goalkeeper_build.sh` compila
el workspace anidado y crea `beijing/beijing_ws/install`. En cambio,
`beijing/scripts/start.sh` calcula `beijing` como raíz y busca allí
`install/setup.bash` y `src/vision/config`.

Por tanto, el árbol versionado es un **staging/overlay**, no un release plano
ejecutable directamente tal como está en Windows. Antes de desplegar, sincronice
los fuentes y scripts con una copia plana de `T2_5v5Demo_Whrg`, compile allí y
verifique las rutas. Este desajuste debe resolverse o empaquetarse explícitamente
antes de llamar a la rama “arranque directo”.

## Mapa del código

### Brain

| Archivo | Responsabilidad |
|---|---|
| [brain.cpp](../../beijing/beijing_ws/src/brain/src/brain.cpp) | Callbacks, modelo de mundo, GameController, predictor, logging y control principal. |
| [brain_tree.cpp](../../beijing/beijing_ws/src/brain/src/brain_tree.cpp) | Registro e implementación de nodos BT. |
| [brain_config.cpp](../../beijing/beijing_ws/src/brain/src/brain_config.cpp) | Carga/validación de parámetros. |
| [brain_data.cpp](../../beijing/beijing_ws/src/brain/src/brain_data.cpp) | Estado compartido del modelo de mundo. |
| [brain_communication.cpp](../../beijing/beijing_ws/src/brain/src/brain_communication.cpp) | Comunicación entre robots. |
| [robot_client.cpp](../../beijing/beijing_ws/src/brain/src/robot_client.cpp) | Construcción y publicación de RPC locomotor. |
| [locator.cpp](../../beijing/beijing_ws/src/brain/src/locator.cpp) | Filtro de partículas y ajuste de pose. |
| [brain_log.cpp](../../beijing/beijing_ws/src/brain/src/brain_log.cpp) | Streams/archivos Rerun. |

Políticas aisladas en headers: predictor del portero, búsqueda, escaneo,
recuperación, obstáculos, path planner, free kick y asistencia. Sus tests están
en [brain/test](../../beijing/beijing_ws/src/brain/test).

### Árboles y configuración

| Ruta | Uso |
|---|---|
| [game.xml](../../beijing/beijing_ws/src/brain/behavior_trees/game.xml) | Partido oficial y estados GC. |
| [demo.xml](../../beijing/beijing_ws/src/brain/behavior_trees/demo.xml) | Demostración/manual. |
| [chase.xml](../../beijing/beijing_ws/src/brain/behavior_trees/chase.xml) | Prueba de persecución. |
| [subtrees](../../beijing/beijing_ws/src/brain/behavior_trees/subtrees) | Rol, localización, búsqueda y recuperación. |
| [config.yaml](../../beijing/beijing_ws/src/brain/config/config.yaml) | Perfil base completo. |
| [config_local.yaml](../../beijing/beijing_ws/src/brain/config/config_local.yaml) | Overrides persistentes de la unidad. |
| [brain launch.py](../../beijing/beijing_ws/src/brain/launch/launch.py) | Selección de árbol/rol/posición/log/com. |

### Visión y GameController

| Archivo | Uso |
|---|---|
| [vision_node.cpp](../../beijing/beijing_ws/src/vision/src/vision_node.cpp) | Sincronía, inferencia, estimadores y publicaciones. |
| [vision.yaml](../../beijing/beijing_ws/src/vision/config/vision.yaml) | Cámara, engines, umbrales y calibración. |
| [game_controller_node.cpp](../../beijing/beijing_ws/src/game_controller/src/game_controller_node.cpp) | Socket UDP, validación y publicación. |
| [GC launch.py](../../beijing/beijing_ws/src/game_controller/launch/launch.py) | Puerto y lista blanca. |
| [RemoteControllerState.msg](../../beijing/beijing_ws/src/booster_ros2_interface/msg/RemoteControllerState.msg) | Contrato del mando. |
| [RpcReqMsg.msg](../../beijing/beijing_ws/src/booster_msgs/msg/RpcReqMsg.msg) | Contrato ROS de petición RPC. |
| [RpcRespMsg.msg](../../beijing/beijing_ws/src/booster_msgs/msg/RpcRespMsg.msg) | Contrato ROS de respuesta RPC. |

### Extensión del portero

| Ruta | Uso |
|---|---|
| [goalkeeper_ball_prediction_policy.h](../../beijing/beijing_ws/src/brain/include/goalkeeper_ball_prediction_policy.h) | Algoritmo matemático testeable. |
| [subtree_goal_keeper_play.xml](../../beijing/beijing_ws/src/brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml) | Selección de modo, bloqueo y patada. |
| [original preservado](../../beijing/beijing_ws/src/brain/behavior_trees/subtrees/subtree_goal_keeper_play_original.xml) | Referencia anterior. |
| [goalkeeper_web](../../beijing/beijing_ws/src/brain/tools/goalkeeper_web) | Servidor, defaults y frontend. |
| [start_goalkeeper_web.sh](../../beijing/scripts/start_goalkeeper_web.sh) | Lanzador del panel. |
| [prepare_goalkeeper_build.sh](../../beijing/scripts/prepare_goalkeeper_build.sh) | Preflight/build/test. |

## Fuentes auditadas

### Locales

- SDK `sdk_release`, rama `feat/T2_pre_release`, commit `5d90357`;
- demo extraído original `T2_5v5Demo_Whrg`;
- código de esta rama `test_goalkeeper`;
- comentarios y contratos de los headers públicos;
- ejecución remota de solo lectura en un T2.

La unidad observada tenía Ubuntu 24.04 ARM64, ROS 2 Kilted, SDK en `/usr/local`,
Rerun C++ 0.20.3 en `/opt/rerun_sdk`, los paquetes del demo instalados y topics
de cámara presentes. En esa inspección no estaban activos los nodos del demo y
existía otro workspace de prueba, posible fuente de contaminación de entorno.

### Pendientes

Los enlaces Feishu entregados requieren una sesión autorizada y no se pudieron
usar como fuente. Por eso este manual no afirma:

- catálogo comercial completo de variantes T2;
- límites mecánicos/eléctricos oficiales;
- procedimiento oficial de batería/carga;
- identificación exacta del paro físico;
- matriz oficial firmware ↔ SDK ↔ imagen de sistema;
- garantías de seguridad del fabricante.

Esos puntos deben completarse con documentación oficial autenticada antes de
operar fuera del entorno RoboCup controlado. No almacene contraseñas del robot
en este repositorio ni en ejemplos de comandos.

Referencias externas pendientes de auditoría autenticada:

- [Documento Booster entregado](https://booster.feishu.cn/docx/IvSOdjoncobt0nxZm5oc9TMnnGf)
- [Wiki Booster entregada](https://booster.feishu.cn/wiki/Uqdrw5hn7iBtWpkrHEec9xsonKd)

La [referencia de CLI de Rerun](https://rerun.io/docs/reference/cli) confirma el
flujo de abrir archivos `.rrd` y el puerto de escucha 9876 en las versiones
actuales. El demo queda fijado en 0.20.3; consulte la ayuda de esa versión antes
de adoptar flags de una versión reciente.
