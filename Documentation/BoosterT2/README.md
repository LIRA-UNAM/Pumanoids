# Manual técnico y de operación — Booster T2

Esta documentación describe exclusivamente el **Booster T2**, el SDK T2 y el
demo RoboCup `T2_5v5Demo_Whrg`. También documenta la extensión de portero que
vive en la rama `test_goalkeeper`. No cubre otras plataformas ni reutiliza su
arquitectura como fuente técnica.

## Para quién es este manual

- operador que debe instalar, arrancar, detener y probar el robot;
- programador que debe entender `vision`, `brain` y GameController;
- integrador que usará el SDK C++, Fast DDS, RPC o ROS 2;
- responsable de campo que necesita localizar fallos sin poner el robot en
  riesgo;
- desarrollador del portero configurable y de su predictor de balón.

## Índice recomendado

1. [Hardware, interfaces y variantes T2](01_HARDWARE_E_INTERFACES.md)
2. [Instalación, compilación y despliegue](02_INSTALACION_Y_DESPLIEGUE.md)
3. [Operación, estados y control remoto](03_OPERACION_Y_CONTROLES.md)
4. [Arquitectura del demo, brain y comportamientos](04_ARQUITECTURA_Y_BRAIN.md)
5. [Visión, localización y Rerun](05_VISION_LOCALIZACION_Y_RERUN.md)
6. [SDK, Fast DDS, RPC y APIs](06_SDK_FASTDDS_RPC_Y_APIS.md)
7. [Configuración y extensión del portero](07_CONFIGURACION_Y_PORTERO.md)
8. [Diagnóstico, pruebas, mapa del código y fuentes](08_DIAGNOSTICO_CODIGO_Y_FUENTES.md)

## Resultado mínimo esperado

Una instalación sana debe cumplir todo esto antes de permitir movimiento:

```text
SDK T2 instalado en /usr/local
        ↓
ROS 2 Kilted + siete paquetes ROS del demo compilados
        ↓
topics de cámara y estado del robot presentes
        ↓
vision publica detecciones y líneas
        ↓
brain recibe visión, odometría, mando y GameController
        ↓
equipo/jugador/rol/red coinciden con el GameController
        ↓
localización válida o comportamiento degradado esperado
        ↓
prueba de movimiento con soporte, zona despejada y parada preparada
```

## Arranque corto

Desde la raíz **plana** del demo, es decir, la carpeta que contiene `src`,
`scripts`, `third_party`, `build`, `install` y `log`:

```bash
cd ~/T2_5v5Demo_Whrg
source /opt/ros/kilted/setup.bash
source install/setup.bash
./scripts/start.sh
```

Para detener los procesos del demo:

```bash
cd ~/T2_5v5Demo_Whrg
./scripts/stop.sh
```

`start.sh` no es un lanzador inocuo: detiene servicios, mata procesos y habilita
el máximo rendimiento de Jetson. Para una primera instalación es preferible
iniciar los componentes uno por uno como se explica en el capítulo 2.

## Configuración que siempre debe comprobarse

| Dato | Archivo | Regla |
|---|---|---|
| Equipo | `brain/config/config.yaml` → `game.team_id` | Debe coincidir con GameController. |
| Jugador | mismo archivo → `game.player_id` | Único dentro del equipo; empieza en 1. |
| Rol | mismo archivo → `game.player_role` | `striker` o `goal_keeper`. |
| IP de respuesta | mismo archivo → `game_control_ip` | IP real de la computadora de GameController. |
| IP permitida | `game_controller/launch/launch.py` → `ip_white_list` | Debe admitir al emisor de UDP. |
| Rerun | `rerunLog.*` | IP/puerto del visor, no del robot. |

En esta rama el perfil actual es equipo `5`, jugador `1`, rol `goal_keeper`,
GameController `10.0.16.150` y visor Rerun `10.0.16.110:9876`. Son valores de
laboratorio, no valores universales.

## Fuentes y nivel de certeza

Las afirmaciones están separadas implícitamente en tres niveles:

- **confirmado por código:** aparece en los fuentes del demo o los encabezados
  del SDK T2;
- **confirmado en un T2:** se observó por SSH en el robot de trabajo;
- **por verificar en hardware:** el SDK expone la API, pero la presencia del
  periférico o servicio depende de la unidad.

Base auditada:

- SDK: rama `feat/T2_pre_release`, commit `5d90357`;
- demo original extraído: `T2_5v5Demo_Whrg`;
- extensión: rama `test_goalkeeper` de este repositorio;
- robot observado: Ubuntu 24.04 ARM64, ROS 2 Kilted y Rerun C++ 0.20.3.

Los dos documentos Feishu entregados inicialmente no pudieron auditarse sin
una sesión autorizada. No se inventan especificaciones ausentes: los huecos se
marcan como pendientes y se indica cómo descubrirlos mediante el SDK.

## Regla de seguridad

Nunca use movimiento, `kDamping`, control articular, patadas o levantado sin:

1. un ayudante junto al robot;
2. soporte o arnés para la primera prueba;
3. suelo despejado y límites físicos conocidos;
4. método de parada ya ensayado;
5. exclusión mutua entre control de alto nivel y control articular.

El envío exitoso de un RPC o mensaje DDS solo confirma comunicación; no prueba
que la acción haya terminado bien.
