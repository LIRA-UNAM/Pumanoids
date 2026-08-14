# Inventario técnico y aceptación

## Base de la rama

- Repositorio: `LIRA-UNAM/Pumanoids`
- Rama base: `beijing_demo`
- Commit base: `99dbd9b3d816f421c528c956fe44f8faeaa103af`
- Rama de trabajo: `test_goalkeeper`
- Workspace del robot: `beijing/beijing_ws`

## Implementación

- `brain.cpp`: parámetros, historial de observaciones, predictor, topics,
  entidades Rerun y correlación decisión-comando-odometría.
- `brain_tree.cpp`: decisiones dinámicas, selector de patada y acción
  `GoalkeeperBlockShot`.
- `goalkeeper_ball_prediction_policy.h`: regresión ponderada, controles de
  calidad, desaceleración e intercepción.
- `subtree_goal_keeper_play.xml`: prioridad de `block_shot` y selector
  Kick/VisualKick.
- `subtree_goal_keeper_play_original.xml`: copia del árbol anterior.
- `tools/goalkeeper_web/`: servidor ROS/HTTP y GUI.
- `factory_defaults.json`: perfil de recuperación protegido con 94 parámetros.
- `config_local.yaml`: ajustes persistentes realizados desde la GUI.
- `start.sh`/`stop.sh`: ciclo de vida del panel integrado al demo.

## Valores y seguridad predeterminados

- predictor desactivado;
- localización requerida al activarlo;
- patada convencional seleccionada;
- restauración sólo carga el formulario y exige una aplicación explícita;
- validación de tipos, límites y relaciones antes de enviar parámetros.

## Configuración de despliegue actual

| Parámetro | Valor |
|---|---:|
| `game.team_id` | `5` |
| `game.player_id` | `1` |
| `game.player_role` | `goal_keeper` |
| `game_control_ip` | `10.0.16.150` |
| `game_controller.ip_white_list` | `10.0.16.150` |
| `rerunLog.server_ip` | `10.0.16.110:9876` |
| `rerunLog.enable_tcp` | `false` |

Cuando cambie la IP del GameController se deben actualizar tanto
`game_control_ip` como `ip_white_list`. Rerun sólo utilizará `server_ip` cuando
`enable_tcp=true`.

## Validación local realizada

- API HTTP: esquema, perfil original, archivos estáticos, aplicación,
  persistencia y rechazo de combinaciones inválidas;
- correspondencia de los 94 parámetros entre GUI, perfil y C++;
- XML y JavaScript válidos;
- scripts Bash válidos;
- prueba aislada del predictor preparada para C++17.

La compilación completa requiere ROS 2, mensajes y SDK instalados en el
Booster; por eso debe ejecutarse el preflight en el robot.

## Criterios de aceptación en el Booster

1. Desde `beijing/`, `bash scripts/prepare_goalkeeper_build.sh` termina con
   `PREPARACION OK`.
2. El panel muestra Brain, GameController y localización.
3. Cambiar `kick.type` actualiza `/brain/goalkeeper/status`.
4. Con el predictor activo y el robot elevado, las muestras y trayectoria se
   actualizan; un balón que se aleja no activa `block_shot`.
5. Sin localización calibrada, el predictor no ordena movimiento con el perfil
   seguro.
6. La prueba en suelo valida el sentido lateral antes de elevar velocidades.
7. La GUI distingue espera de Brain (`waiting_command`) de espera física
   (`waiting_motion`) y registra los tiempos resultantes en JSONL.

## Reversión

La reversión recomendada se realiza desde la GUI:
**Ayuda y restauración → Cargar todos los valores originales → Aplicar y
guardar**. Para volver también al árbol anterior, el XML original está
conservado junto al activo; hacerlo requiere recompilar `brain`.
