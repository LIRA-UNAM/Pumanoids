# Manifiesto de implementación del portero

Estado: implementación integrada en la rama `test_goalkeeper`; validaciones
locales terminadas y compilación/aceptación física pendientes en el Booster.

## Base

- Rama base: `beijing_demo`
- Commit base: `99dbd9b3d816f421c528c956fe44f8faeaa103af`
- Rama resultante: `test_goalkeeper`
- Demo desplegable: `beijing/beijing_ws`

## Alcance completado

| Área | Implementación | Estado |
|---|---|---|
| GUI | 102 parámetros ROS con límites, descripciones y ayuda | Terminado |
| Aplicación | Servicio atómico de parámetros de `/brain_node` | Terminado |
| Persistencia | Escritura atómica de `config_local.yaml` | Terminado |
| Restauración | Perfil original protegido de 102 valores | Terminado |
| Kick | Parámetros dinámicos y detención segura | Terminado |
| VisualKick | Selector, kV1/kV2, tiempos y protecciones | Terminado |
| Predicción | Ajuste ponderado, velocidad, calidad, fricción y cruce | Terminado |
| Seguridad | Predictor apagado y localización requerida por defecto | Terminado |
| Árbol | Decisión prioritaria `block_shot` e intercepción | Terminado |
| Marcha urgente | Bloqueo lateral puro; sin piso artificial en X/giro | Terminado |
| Observabilidad | GUI, dos topics ROS, JSONL de reacción y dos entidades Rerun | Terminado |
| Inicio/parada | Integrado en scripts del demo Beijing | Terminado |
| Documentación | README, operación, parámetros, red y aceptación | Terminado |

## Configuración operativa actual

| Ajuste | Valor |
|---|---:|
| Equipo | `5` |
| Jugador | `1` |
| Rol | `goal_keeper` |
| GameController | `10.0.16.150` |
| Lista blanca del receptor | `10.0.16.150` |
| Rerun Viewer | `10.0.16.110:9876` |
| Streaming Rerun | Desactivado hasta poner `enable_tcp=true` |

## Validaciones realizadas sin robot

- API web: página, esquema, perfil original, aplicación, persistencia y rechazo
  de combinaciones inválidas.
- Correspondencia GUI/C++/perfil: 102/102 parámetros.
- Accesos `goalkeeper.*`: ninguno sin declaración.
- XML, Python, JavaScript y scripts Bash: sintaxis válida.
- Predictor aislado preparado como prueba C++17.
- Launch del GameController: sintaxis Python válida.
- `game_control_ip` e `ip_white_list`: ambos en `10.0.16.150`.

No se afirma que el paquete ROS completo compile hasta probarlo con los
headers, mensajes y SDK instalados en el robot.

## Procedimiento pendiente en el Booster

```bash
cd ~/Pumanoids
git switch test_goalkeeper
git pull
cd beijing
bash scripts/prepare_goalkeeper_build.sh
./scripts/start.sh role:=goal_keeper
```

Después abrir `http://IP_DEL_ROBOT:8088`.

## Criterios de aceptación

1. El preflight termina con `PREPARACION OK`.
2. La GUI reporta Brain, GameController y localización.
3. Cambiar patada se refleja en `/brain/goalkeeper/status`.
4. Elevado, el predictor muestra muestras y trayectoria para un balón entrante.
5. Un balón que se aleja no activa `block_shot`.
6. Sin `odom_calibrated`, el predictor no ordena movimiento.
7. La prueba en suelo confirma el sentido lateral antes de subir velocidad.
8. La GUI registra decisión → comando → movimiento y permite diferenciar
   retardo de Brain de retardo físico del T2 con el predictor apagado.

## Reversión

- Perfil original: `goalkeeper.prediction.enabled=false`; perfil recomendado
  persistido: `true` y requiere localización.
- Patada original: `goalkeeper.kick.type=default`.
- Restauración completa: **Ayuda y restauración → Cargar todos los valores
  originales → Aplicar y guardar**.
- Árbol anterior: `subtree_goal_keeper_play_original.xml`.
