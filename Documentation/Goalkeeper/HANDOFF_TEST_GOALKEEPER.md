# Traspaso técnico de `test_goalkeeper`

Este documento permite continuar el desarrollo, desplegar la rama en varios
Booster T2 y fusionarla con `beijing_demo` sin perder comportamiento ni mezclar
configuraciones propias de cada robot.

## 1. Estado auditado

Fecha de la auditoría: **2026-08-15**.

| Referencia | Commit auditado |
|---|---|
| Base común de las ramas | `99dbd9b3d816f421c528c956fe44f8faeaa103af` |
| `origin/beijing_demo` | `5f9329e9282cf09e9179a52d2e6460e92cdcd6ef` |
| Implementación `test_goalkeeper` anterior a este documento | `adfb487287f397fdbb0f9713f1bf024e412b809a` |

La comparación directa de los extremos mostraba 48 archivos distintos,
aproximadamente 6852 líneas añadidas y 176 eliminadas. La mayor parte corresponde
a documentación, GUI, predictor y telemetría; no debe interpretarse como una
reescritura total del demo.

## 2. Objetivo y alcance de la rama

La rama agrega al demo del Booster T2:

- portero parametrizable sin recompilar cada ajuste;
- selección entre `Kick` convencional y `RLVisionKick`;
- predictor temporal de la trayectoria del balón;
- estado `block_shot` y bloqueo lateral urgente;
- persecución y despeje posterior al bloqueo;
- rechazo opcional de falsos balones fuera del campo;
- panel web con 102 parámetros, restauración y persistencia;
- telemetría persistente y visualización 2D;
- métricas de decisión, comando y movimiento;
- pruebas funcionales del predictor, web y compatibilidad con la SDK;
- documentación integral del Booster T2.

No agrega una acción de lanzarse o tirarse al suelo. El portero bloquea caminando,
principalmente en dirección lateral. Tampoco convierte el filtro de balón en un
sistema para impedir que el robot salga físicamente del campo.

## 3. Layout de desarrollo y ejecución

Existen dos niveles deliberados:

```text
Pumanoids/beijing/
├── beijing_ws/src/       # fuentes versionados o staging
├── build/                # build ejecutable plano
├── install/              # instalación que carga start.sh
├── scripts/
└── goalkeeper_logs/
```

`start.sh` siempre se ejecuta desde `Pumanoids/beijing` y carga:

```bash
source ./install/setup.bash
```

`prepare_goalkeeper_build.sh` toma los fuentes de `beijing_ws/src`, pero usa
`--build-base beijing/build` y `--install-base beijing/install`. No se debe
cambiar `start.sh` para cargar `beijing_ws/install`: volvería a crear dos
instalaciones ROS diferentes y podría ejecutar un binario antiguo.

Preparación validada:

```bash
cd ~/Pumanoids/beijing
bash scripts/prepare_goalkeeper_build.sh
```

El script comprueba sintaxis, correspondencia de los 102 parámetros, XML,
compila `brain`, ejecuta 12 pruebas funcionales y confirma que el binario plano
contiene `goalkeeper.prediction.reject_outside_field`.

## 4. Precedencia de configuración

El valor efectivo de un parámetro puede venir de varias capas. De menor a mayor
precedencia:

1. valor declarado en C++ mediante `declare_parameter`;
2. `brain/config/config.yaml`, perfil base completo;
3. `brain/config/config_local.yaml`, perfil persistente de la GUI;
4. argumentos de launch para `team`, `id`, `role`, `pos`, `disable_log` y
   `disable_com`;
5. cambios ROS aplicados en vivo desde la web.

Archivos canónicos:

- perfil base: `beijing/beijing_ws/src/brain/config/config.yaml`;
- perfil operativo: `beijing/beijing_ws/src/brain/config/config_local.yaml`;
- original protegido: `brain/tools/goalkeeper_web/factory_defaults.json`;
- esquema, rangos y ayuda: `brain/tools/goalkeeper_web/server.py`.

La web distingue dos operaciones:

- **Aplicar**: modifica el proceso `brain_node` actual; se pierde al reiniciar;
- **Aplicar y guardar**: además actualiza `config_local.yaml` para el siguiente
  arranque.

Si `brain_node` está detenido, `/api/config` devuelve los valores guardados con
`live: false`. En ese estado solo **Aplicar y guardar** tiene sentido. Cuando el
nodo está disponible, `live: true` permite aplicar inmediatamente.

## 5. Identidad, red y valores compartidos

### 5.1 Valores actuales de esta rama

| Ajuste | Valor actual | Alcance |
|---|---:|---|
| `game.team_id` | `5` | Común al equipo |
| launch `team` | default `5` | Común; puede sobrescribir YAML |
| `game.player_id` | `1` | **Único por robot** |
| launch `id` | vacío | Usa el YAML; se puede pasar `id:=N` |
| `game.player_role` | `goal_keeper` | Normalmente único por función |
| `game.number_of_players` | `5` | Común al equipo |
| `game.field_type` | `robo_league` | Común al partido |
| `game.player_start_pos` | `right` | Por robot o por sesión |
| `game_control_ip` | `10.0.16.150` | Común si todos usan el mismo GC |
| `ip_white_list` | `10.0.16.150` | Común y debe coincidir con el GC |
| `rerunLog.server_ip` | `10.0.16.110:9876` | Común si comparten viewer |
| `rerunLog.enable_tcp` | `false` | Por sesión/red |
| `robot.robot_height` | `1.28146` | Común a T2; no copiar a K1 |

### 5.2 Qué puede compartirse entre robots

Puede versionarse igual para unidades T2 equivalentes:

- parámetros del predictor y del bloqueo;
- parámetros de Kick/VisualKick si las unidades tienen la misma versión de SDK;
- dimensiones y tipo de campo;
- número de equipo y cantidad de jugadores;
- IP del GameController y whitelist si están en la misma red;
- árbol de comportamiento, GUI, pruebas y scripts;
- perfil Fast DDS cuando todos usan la misma interfaz de red.

### 5.3 Qué debe revisarse por robot

No copiar ciegamente entre unidades:

- `game.player_id`: debe ser único dentro del equipo;
- `game.player_role`: solo una unidad debe actuar como portero normal;
- `game.player_start_pos`: depende del lado y la colocación;
- calibración intrínseca/extrínseca de cámara y `/opt/booster/vision.yaml`;
- compensaciones de visión y `vision_local.yaml`;
- escala, orientación y alineación de odometría si difieren físicamente;
- IP propia del robot, interfaz Wi-Fi/Ethernet y rutas;
- límites que se ajustaron a desgaste, fricción o respuesta de una unidad;
- versión real de la SDK y disponibilidad de VisualKick.

Ejemplos para varios robots del equipo 5:

```bash
# Portero, toma id=1 del YAML
./scripts/start.sh role:=goal_keeper team:=5

# Jugador 2
./scripts/start.sh role:=striker team:=5 id:=2

# Jugador 3 en el otro lado de entrada
./scripts/start.sh role:=striker team:=5 id:=3 pos:=left
```

Dos robots con el mismo par `(team_id, player_id)` producen ambigüedad de estado,
posesión y comunicación. Debe comprobarse antes de cada prueba multi-robot.

## 6. Parámetros del portero

La GUI contiene 102 parámetros distribuidos así:

| Grupo | Cantidad | Responsabilidad |
|---|---:|---|
| Bloqueo reactivo | 18 | READY, cobertura y modo attack/guard |
| Persecución | 8 | Aproximación al balón y obstáculos |
| Ajuste y posesión | 10 | Alineación, claim y liderazgo |
| Cámara | 10 | Seguimiento y barrido de cabeza |
| Patada | 20 | Kick, VisualKick y seguridades |
| Predicción | 24 | Muestras, ajuste, calidad y horizonte |
| Bloqueo predictivo | 12 | Velocidades y reacción a tiros urgentes |

Todos aparecen en `server.py`, se declaran en `brain.cpp` y se comprueban en
`prepare_goalkeeper_build.sh`. Añadir un parámetro exige actualizar al menos:

1. declaración C++;
2. lectura en C++ o árbol;
3. `config.yaml`;
4. `config_local.yaml` si debe tener override;
5. esquema web;
6. `factory_defaults.json`;
7. prueba o preflight correspondiente.

## 7. Perfil actual frente al original

El perfil vigente fue actualizado después de la sesión de campo del 2026-08-15.
La evidencia, protocolo A/B y restauración del perfil inmediatamente anterior
están en `AJUSTE_PRUEBA_20260815.md`. La tabla siguiente refleja el estado
actual, no el perfil urgente usado durante esa sesión.

| Parámetro | Original | Actual | Efecto principal |
|---|---:|---:|---|
| `goalkeeper.blocking.vy_limit` | 0.9 | 1.3 | Mayor velocidad lateral en cobertura |
| `goalkeeper.blocking.position_gain` | 1.0 | 1.5 | Convierte antes el error en velocidad |
| `goalkeeper.chase.vy_limit` | 0.3 | 1.5 | Aproximación lateral más rápida |
| `goalkeeper.chase.safe_distance` | 0.6 | 0.5 | Rodea el balón con menor radio |
| `goalkeeper.claim.max_ball_range` | 1.5 | 3.0 | Permite abandonar cobertura desde más lejos |
| `goalkeeper.claim.lateral_margin` | 0.5 | 1.0 | Amplía la zona lateral reclamable |
| `goalkeeper.kick.alignment_tolerance` | 1.5708 | 0.78 | Exige mejor alineación antes del despeje |
| `goalkeeper.kick.default.speed_limit` | 1.2 | 1.5 | Kick convencional más rápido |
| `goalkeeper.kick.default.min_msec` | 600 | 500 | Reduce duración mínima del Kick |
| `goalkeeper.kick.default.exit_range` | 1.0 | 1.5 | Amplía rango de salida del estado Kick |
| `goalkeeper.kick.default.ball_move_threshold` | 0.30 | 0.25 | Confirma antes que el balón salió |
| `goalkeeper.kick.visual.pre_delay_msec` | 1000 | 200 | Reduce espera antes de VisualKick |
| `goalkeeper.kick.visual.post_delay_msec` | 1000 | 450 | Reduce espera posterior |
| `goalkeeper.prediction.enabled` | false | true | Activa `block_shot` predictivo |
| `goalkeeper.prediction.reject_outside_field` | false | true | Filtra falsos balones fuera del campo |
| `goalkeeper.prediction.max_time_to_block` | 2.5 | 3.0 | Acepta amenazas algo más lejanas |
| `goalkeeper.prediction.block.vx_limit` | 0.7 | 0.65 | Conserva una corrección diagonal moderada |
| `goalkeeper.prediction.block.vy_limit` | 1.0 | 1.5 | Bloqueo lateral predictivo más rápido |
| `goalkeeper.prediction.block.reaction_margin_sec` | 0.12 | 0.10 | Vuelve al perfil inicial medido |
| `goalkeeper.prediction.block.urgent_time_sec` | 1.2 | 0.0 | Desactiva el sobrecontrol urgente tardío |
| `obstacle_avoidance.chase_ao_safe_dist` | 1.5 | 1.0 | Reduce separación durante persecución |

Cambiar varias velocidades simultáneamente dificulta atribuir resultados. En
pruebas de campo debe alterarse una familia por vez y conservar el log JSONL.

## 8. Filtro de objetos fuera del campo

Los parámetros son:

```yaml
goalkeeper.prediction.field_margin: 0.5
goalkeeper.prediction.reject_outside_field: true
```

Tras calibrar localización, se rechaza una detección si:

```text
abs(ball_x) > field_length / 2 + field_margin
o
abs(ball_y) > field_width / 2 + field_margin
```

El margen de 0.5 m evita rechazar un balón real muy próximo a la línea por ruido
de pose. Antes de `odom_calibrated`, el filtro de selección permanece inactivo
para no perder el balón durante entrada o calibración. El predictor limpia su
historial cuando recibe una posición de campo físicamente imposible.

En la web deben observarse:

- `field_filter_enabled`;
- `field_filter_localization_ready`;
- `field_filter_rejected_count`;
- última posición y confianza rechazadas.

Este filtro no limita la trayectoria del robot. Las funciones de borde y
planificación son mecanismos diferentes.

Además, sólo para `goal_keeper` con predictor activo, una pelota observada hace
menos de 250 ms impide seleccionar otro candidato separado más de
`max_sample_jump` (actualmente 0.8 m). Esto evita cambios instantáneos entre
objetos dentro del campo. La telemetría publica `ball_jump_rejected_count` y la
distancia/posición del último rechazo. Striker y los demás roles conservan el
selector original por máxima confianza.

## 9. Predictor y decisión de bloqueo

Flujo simplificado:

1. visión produce candidatos de balón;
2. brain filtra confianza, saltos y límites de campo;
3. conserva muestras recientes con marca temporal;
4. ajusta posición contra tiempo, ponderando más lo reciente;
5. valida velocidad, componente hacia la portería, R² y residual;
6. simula desaceleración y calcula cruce con la línea defensiva;
7. comprueba postes, margen y ventana temporal;
8. publica diagnóstico y selecciona `block_shot`;
9. `BlockPredictedShot` ordena movimiento normal X/Y; el código urgente queda
   disponible, pero el perfil actual lo desactiva con `urgent_time_sec=0`;
10. después del bloqueo puede habilitar claim, Chase, Adjust y Kick.

`reaction_margin_sec` no retrasa intencionalmente: representa tiempo que ya no
está disponible para caminar. Si se reactiva la urgencia, aumentarlo hace que el
sistema sature antes. En la sesión 16:51, el movimiento alineado urgente tardó
345 ms frente a 85 ms normal, por lo que no debe reactivarse hasta terminar la
comparación documentada.

## 10. Patadas

`goalkeeper.kick.type` selecciona:

- `default`: nodo `Kick`, crab-walk convencional con control de duración y
  desplazamiento del balón;
- `visual`: `RLVisionKick` de la SDK, versión `kV1` o `kV2`.

Los parámetros de un tipo no afectan al otro. Antes de cambiar a VisualKick hay
que confirmar que la SDK de esa unidad soporta la versión seleccionada. La rama
actual usa `default` como perfil recomendado.

No existe un parámetro llamado PowerShoot conectado al portero. Los valores de
potencia que no llegan al RPC utilizado no deben documentarse como funcionales.

## 11. Cambios de código por área

### Mapa de archivos de mayor impacto

| Archivo | Cambio y motivo | Riesgo al fusionar |
|---|---|---|
| `brain/src/brain.cpp` | parámetros, observaciones, predictor, filtro, estado y telemetría | Alto: archivo central y extenso |
| `brain/src/brain_tree.cpp` | nodos BT, claim, bloqueo, Chase, Adjust y Kick | Alto: decisiones y movimiento |
| `brain/src/robot_client.cpp` | instrumentación y comandos locomotores | Medio: contrato con SDK/RPC |
| `brain/src/brain_log.cpp` | entidades y métricas Rerun | Medio: observabilidad |
| `brain/include/brain.h` | estado y métodos nuevos | Alto si cambia `brain.cpp` |
| `brain/include/brain_data.h` | datos compartidos de predicción/decisión | Alto por ABI interna |
| `brain/include/brain_tree.h` | declaraciones de nodos BT | Medio |
| `brain/include/goalkeeper_ball_prediction_policy.h` | algoritmo matemático aislado y testeable | Bajo si se conserva como unidad |
| `brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml` | flujo activo del portero | Alto: orden de prioridades |
| `subtree_goal_keeper_play_original.xml` | copia inmutable de referencia | No editar durante merge |
| `brain/config/config.yaml` | defaults, identidad y 102 parámetros | Conflicto textual previsto |
| `brain/config/config_local.yaml` | perfil operativo persistente | Alto si se comparte entre unidades |
| `brain/launch/launch.py` | overrides de identidad/rol/lado | Conflicto textual previsto |
| `brain/tools/goalkeeper_web/` | servidor, GUI, defaults y ayuda | Bajo respecto a upstream, alto funcionalmente |
| `brain/CMakeLists.txt` y `package.xml` | instalación web, tests y dependencias Python | Medio al cambiar build |
| `vision/config/vision.yaml` | supuesto de un solo balón | Medio: percepción |
| `vision/src/vision_node.cpp` | recorrido completo de detecciones | Medio: selección del balón |
| `scripts/prepare_goalkeeper_build.sh` | despliegue staging → install plano | Alto: determina qué binario corre |
| `scripts/start.sh` | arranque de web y layout plano | Conflicto textual previsto |
| `scripts/start_goalkeeper_web.sh` | proceso web desacoplado y configuración | Bajo, pero necesario para GUI |
| `scripts/stop.sh` | cierre completo de web | Bajo; evita contexto ROS inválido |

### Brain y datos

- declaración y lectura dinámica de parámetros;
- historial de observaciones y resultado de predicción;
- decisión, razón, edades y métricas de reacción;
- rechazo de balón fuera del campo;
- publicación de `/brain/goalkeeper/status` y decisión;
- timestamps decisión → comando → movimiento.

### Árbol de comportamiento

- se preservó `subtree_goal_keeper_play_original.xml`;
- el subárbol activo añade modos attack/guard;
- integra bloqueo predictivo, cobertura, claim, Chase, Adjust y patada;
- después de un bloqueo puede reclamar el balón durante una ventana acotada.

### Control locomotor

- instrumentación de comandos y detección de movimiento físico;
- velocidades específicas de bloqueo y compensación de zona muerta lateral;
- no se añadió una API de dive.

### GUI web

- servidor HTTP/ROS 2 y panel estático;
- perfiles original y recomendado;
- aplicación temporal/persistente;
- lectura desde YAML cuando brain está detenido;
- logs JSONL, estado, tabla y campo 2D;
- reinicio desacoplado con `setsid`;
- parada forzada y específica porque `rclpy` interceptaba SIGTERM y dejaba el
  HTTP vivo con el contexto ROS inválido.

### Build y arranque

- `prepare_goalkeeper_build.sh` despliega staging en el `install` plano;
- `start.sh` inicia la web después de visión, brain y GameController;
- `stop.sh` detiene también la web;
- launch de brain acepta `team` default 5 e `id` opcional;
- se añadieron dependencias de ejecución `rclpy` y `rcl_interfaces`.

### Visión

- `single_ball_assumption` cambió de `false` a `true`;
- se eliminó un `break` al recorrer detecciones de balón.

Esta combinación debe revisarse si se cambia el postproceso: con más de un
candidato residual, el orden de detecciones puede afectar qué mensaje termina
publicándose. El brain vuelve a seleccionar por confianza en su propia lista,
pero no debe asumirse que ambas capas resuelven múltiples balones igual.

## 12. Web, logs y observabilidad

Panel:

```text
http://IP_DEL_ROBOT:8088
```

Archivos:

```text
beijing/goalkeeper_web.log
beijing/goalkeeper_logs/goalkeeper_telemetry_*.jsonl
beijing/brain.log
beijing/vision.log
beijing/game_controller.log
```

Endpoints principales:

| Endpoint | Uso |
|---|---|
| `/api/schema` | parámetros, rangos y ayuda |
| `/api/config` | valores live o persistidos |
| `/api/status` | última decisión y diagnóstico |
| `/api/telemetry` | historial en memoria y ruta del JSONL |
| `/api/log/download` | descarga del log activo |
| `/api/apply` | aplicar y opcionalmente persistir |

Si la página aparece sin campos, revisar primero:

```bash
curl http://127.0.0.1:8088/api/config
pgrep -af goalkeeper_web/server.py
```

Debe existir una sola instancia. Con brain detenido es correcto recibir
`live:false`; no es correcto recibir HTTP 503 o `rcl node's context is invalid`.

## 13. Cambios acumulados por commit

| Commit | Propósito |
|---|---|
| `e87a435` | base configurable del portero y laboratorio |
| `de74b81` | identidad y red de despliegue |
| `07df921` | documentación de configuración |
| `0748e18` | manual integral Booster T2 |
| `e29b17a` | telemetría de laboratorio |
| `c85840a` | retorno al flujo con GameController |
| `9315fcc` | evita autenticación duplicada de servicios |
| `002945d` | diagnóstico de reacción y despeje |
| `8258f2b` | latencia decisión → movimiento |
| `f4313dd` | predictor, bloqueo urgente y perfil medido |
| `9292519` | validación robusta bajo ROS Kilted |
| `fbb1760` | comprobación correcta de archivos instalados |
| `a898bb9` | despliegue plano y web resiliente offline |
| `adfb487` | overrides launch `team`/`id` |

## 14. Riesgos y límites conocidos

- El predictor depende de localización válida para actuar de forma segura.
- `reject_outside_field` puede rechazar el balón real si la pose está mal y se
  marca erróneamente como calibrada.
- La velocidad lateral física puede ser inferior al comando y limitar tiros rápidos.
- Un margen de reacción mayor no aumenta la velocidad mecánica.
- Chase y claim agresivos pueden alejar al portero de la línea.
- VisualKick depende de versión y estado de la SDK.
- Los linters globales del demo tienen deuda heredada; la preparación ejecuta
  las 12 pruebas funcionales, no usa el resultado de `uncrustify` como criterio
  operativo.
- Los logs son datos de prueba, no deben agregarse al repositorio.
- No ejecutar dos instalaciones (`beijing/install` y `beijing_ws/install`) como
  overlays simultáneos.
- No probar movimiento con el robot acostado o sin control de emergencia.

## 15. Fusión con `beijing_demo`

La simulación de merge desde la base común predice conflictos textuales en:

1. `brain/config/config.yaml`;
2. `brain/launch/launch.py`;
3. `scripts/start.sh`.

`game_controller/launch/launch.py` fue modificado en ambas ramas, pero los
extremos actuales contienen la misma whitelist `10.0.16.150`, por lo que no
presenta diferencia tip-to-tip.

### 15.1 Resolución recomendada por archivo

#### `config.yaml`

Conservar toda la sección `goalkeeper` y parámetros auxiliares. Resolver
manualmente identidad y rol:

- upstream: equipo 0, jugador 2, striker;
- esta rama: equipo 5, jugador 1, goal_keeper.

No escoger el archivo completo con `--ours` o `--theirs`. La identidad debe
separarse del código común y revisarse por robot.

#### `brain/launch/launch.py`

Conservar:

- argumento `team`, default `5` por decisión actual del equipo;
- argumento `id`, default vacío;
- overrides `role`, `pos`, `disable_log` y `disable_com`;
- búsqueda segura de configuración de visión.

Upstream usa `team` default `0`; aceptarlo cambiaría silenciosamente el equipo
cuando `start.sh` no recibe `team:=...`.

#### `scripts/start.sh`

Conservar para este despliegue:

```bash
source ./install/setup.bash
```

También conservar arranque de la web y `sudo` para el servicio de speech.
Upstream apunta a `beijing_ws/install`; mezclarlo con el build plano volvería a
ejecutar otro binario.

### 15.2 Cambios semánticos sin conflicto textual

Una fusión puede completar sin conflicto y aun cambiar el robot:

- `game.xml`: upstream reemplaza `SetVelocity` durante FreeKick STOP por
  `GoToFreekickPosition` de ataque/defensa. Se aplicará automáticamente porque
  esta rama no cambió ese archivo desde la base. Debe probarse en SET/STOP.
- `new_start.sh`: upstream lo añade con defaults equipo 0, jugador 2 y striker.
  No inicia la web y puede contradecir esta rama; revisar antes de usar.
- `.gitignore`: upstream añade `*.log`; es deseable conservarlo.
- identidad en YAML y launch: aunque compile, una elección equivocada puede
  duplicar IDs o desconectar GameController.
- visión: conservar conscientemente el supuesto de un solo balón o volver al
  comportamiento upstream después de pruebas.

### 15.3 Procedimiento seguro de integración

Hacer la fusión en una rama temporal, no directamente en la unidad de campo:

```bash
git fetch origin
git switch -c integration/goalkeeper-beijing test_goalkeeper
git merge --no-commit origin/beijing_demo
git status
```

Resolver manualmente los tres archivos indicados y revisar también `game.xml`,
`new_start.sh`, visión y red. Después:

```bash
git diff --check
python3 -m py_compile \
  beijing/beijing_ws/src/brain/launch/launch.py \
  beijing/beijing_ws/src/brain/tools/goalkeeper_web/server.py
cd beijing
bash scripts/prepare_goalkeeper_build.sh
```

Comprobaciones posteriores en el robot, sin arrancar movimiento:

```bash
source /opt/ros/kilted/setup.bash
source install/setup.bash
ros2 launch brain launch.py --show-args
strings install/brain/lib/brain/brain_node | \
  grep goalkeeper.prediction.reject_outside_field
curl http://127.0.0.1:8088/api/config
```

No aceptar la fusión hasta confirmar:

- `team` default 5;
- `id` opcional y único por robot;
- 12/12 pruebas funcionales;
- 102 parámetros en `/api/config`;
- filtro fuera del campo presente en el binario plano;
- una sola instancia web;
- GameController recibido desde la IP permitida;
- comportamiento FreeKick revisado.

## 16. Checklist para continuar desarrollo

Antes de modificar:

- registrar commit exacto y configuración usada;
- copiar logs de la sesión anterior;
- confirmar ID, rol, lado, campo y red;
- cambiar una familia de parámetros por prueba.

Antes de desplegar:

- `git diff --check`;
- ejecutar prueba web y del predictor;
- compilar con `prepare_goalkeeper_build.sh`;
- confirmar que `start.sh` carga `./install/setup.bash`;
- no iniciar nodos si el robot no está vertical y despejado.

Después de una sesión:

- descargar JSONL y logs de brain/visión/GC;
- anotar goles, tiros, falsos positivos y configuración;
- distinguir latencia de decisión, latencia de comando y velocidad física;
- no ajustar predictor y locomoción simultáneamente sin una hipótesis medible.

## 17. Documentos relacionados

- `Documentation/Goalkeeper/README.md`: operación del portero;
- `Documentation/Goalkeeper/PORTERO_GUI_PREDICCION.md`: GUI y predictor;
- `Documentation/Goalkeeper/IMPLEMENTATION.md`: inventario técnico;
- `Documentation/BoosterT2/02_INSTALACION_Y_DESPLIEGUE.md`: instalación;
- `Documentation/BoosterT2/05_VISION_LOCALIZACION_Y_RERUN.md`: localización;
- `Documentation/BoosterT2/07_CONFIGURACION_Y_PORTERO.md`: parámetros;
- `Documentation/BoosterT2/08_DIAGNOSTICO_CODIGO_Y_FUENTES.md`: diagnóstico.

Este documento debe actualizarse cuando cambien la base de `beijing_demo`, el
perfil de 102 parámetros, las IP, la identidad del robot o la estrategia de
despliegue.
