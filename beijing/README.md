# Demo Beijing — rama `test_goalkeeper`

Esta rama se creó desde `beijing_demo` para probar el portero configurable en
el Booster T2. Mantiene el demo de Beijing y agrega:

- panel web en `http://IP_DEL_ROBOT:8088` con 93 parámetros;
- aplicación temporal o persistente de parámetros ROS 2;
- restauración del perfil original protegido;
- selector entre Kick convencional y VisualKick;
- predictor de trayectoria y comportamiento `block_shot`;
- estado por topics ROS y trayectoria/punto de cruce en Rerun.

El comportamiento seguro/original queda seleccionado inicialmente:
`goalkeeper.mode=attack`, `goalkeeper.kick.type=default` y
`goalkeeper.prediction.enabled=false`.

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
[Documentation/Goalkeeper](../Documentation/Goalkeeper/README.md).
