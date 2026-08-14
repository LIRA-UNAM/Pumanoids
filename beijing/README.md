# Demo Beijing — rama `test_goalkeeper`

> Manual integral del Booster T2: [Documentation/BoosterT2](../Documentation/BoosterT2/README.md).

Esta rama se creó desde `beijing_demo` para probar el portero configurable en
el Booster T2. Mantiene el demo de Beijing y agrega:

- panel web en `http://IP_DEL_ROBOT:8088` con 94 parámetros;
- aplicación temporal o persistente de parámetros ROS 2;
- restauración del perfil original protegido;
- selector entre Kick convencional y VisualKick;
- predictor de trayectoria y comportamiento `block_shot`;
- medición de latencia decisión → comando → movimiento físico, aun con el
  predictor desactivado;
- estado por topics ROS y trayectoria/punto de cruce en Rerun.

El comportamiento seguro/original queda seleccionado inicialmente:
`goalkeeper.mode=attack`, `goalkeeper.kick.type=default` y
`goalkeeper.prediction.enabled=false`.

## Configuración actual del robot y la red

Esta rama está preparada para el portero del **equipo 5**, jugador **1**:

| Ajuste | Valor | Archivo |
|---|---:|---|
| Número de equipo | `5` | `beijing_ws/src/brain/config/config.yaml` → `game.team_id` |
| Número de jugador | `1` | `beijing_ws/src/brain/config/config.yaml` → `game.player_id` |
| Rol | `goal_keeper` | `beijing_ws/src/brain/config/config.yaml` → `game.player_role` |
| IP del GameController | `10.0.16.150` | `config.yaml` → `game_control_ip` |
| IP admitida por el receptor | `10.0.16.150` | `game_controller/launch/launch.py` → `ip_white_list` |
| Equipo con Rerun Viewer | `10.0.16.110:9876` | `config.yaml` → `rerunLog.server_ip` |

La IP del GameController aparece en **dos archivos** deliberadamente:

- `game_control_ip` indica al brain dónde está el GameController;
- `ip_white_list` hace que el nodo receptor descarte datagramas enviados desde
  cualquier otra IP.

Si cambia la computadora del GameController, actualizar ambos valores con la
misma IP. Para aceptar temporalmente más de un origen se puede iniciar el nodo
con una lista separada por comas:

```bash
ros2 launch game_controller launch.py \
  ip_white_list:="10.0.16.150,OTRA_IP"
```

La transmisión de Rerun está actualmente desactivada mediante
`rerunLog.enable_tcp=false`. Para verla por red, ponerla en `true`, confirmar
que Rerun escucha en `10.0.16.110:9876` y que robot y computadora tienen ruta
entre sí.

Antes de un partido, comprobar que el número de equipo coincide exactamente
con el configurado en el GameController y que no existe otro jugador `1` en el
mismo equipo.

## Preparar el robot

Desde la raíz del repositorio:

```bash
cd beijing
bash scripts/prepare_goalkeeper_build.sh
```

El script verifica ROS/SDK, la GUI, el árbol de comportamiento, compila
`brain`, ejecuta sus pruebas y comprueba los archivos instalados.

## Ejecutar

```bash
cd beijing
./scripts/start.sh role:=goal_keeper
```

Luego abrir desde un equipo en la misma red:

```text
http://IP_DEL_ROBOT:8088
```

La sección **Ayuda y restauración** del panel explica cada parámetro y el flujo
correcto para probar, aplicar y guardar. Para detener todo:

```bash
cd beijing
./scripts/stop.sh
```

## Seguridad

No activar el predictor por primera vez con el robot libre en el campo. Probar
primero elevado o en `INITIAL`, confirmar `localization_ready=true` y validar
el sentido lateral. La configuración predeterminada no permite que el
predictor ordene un bloqueo sin localización calibrada.

La documentación completa está en
[Documentation/Goalkeeper](../Documentation/Goalkeeper/README.md), incluyendo
el [manual completo de GUI y predicción](../Documentation/Goalkeeper/PORTERO_GUI_PREDICCION.md)
y el [manifiesto de implementación](../Documentation/Goalkeeper/IMPLEMENTACION_PORTERO_MANIFIESTO.md).
