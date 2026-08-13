# Visión, localización y Rerun

## Tres procesos diferentes

Conviene separar conceptos que suelen llamarse “calibración”:

1. **calibración de cámara/extrínsecos:** estima la transformación cámara-cabeza
   y se guarda en `vision.yaml` o `/opt/booster/vision.yaml`;
2. **localización inicial:** estima `x`, `y` y orientación del robot dentro del
   campo usando marcas visuales y odometría;
3. **actualización de pose:** durante el juego integra odometría y vuelve a
   corregir con marcas.

La localización de partido no reescribe la calibración física de cámara.

## Cadena de visión

La configuración actual usa:

- cámara `d-robotics`;
- detector TensorRT `model/T2_0804_digua.engine`;
- clases Ball, Goalpost, Person, cruces L/T/X, PenaltyPoint, Opponent y BRMarker;
- confianza global 0.2 y umbrales por clase;
- segmentación desactivada porque el engine indicado no está habilitado en
  JetPack 7.2;
- profundidad desactivada para los estimadores de balón y persona en el perfil
  actual;
- pose timestamped `/head_pose_stamped`, con sincronía máxima 40 ms;
- imagen `544 × 448`.

`vision_node` publica:

```text
/booster_vision/detection
/booster_vision/line_segments    # solo si la segmentación está habilitada
/booster_vision/ball
/booster_vision/t_head2base
```

Que `/booster_vision/ball` exista no implica que brain conozca un balón actual:
revise timestamp, confianza, rango y timeout.

`vision.yaml: use_depth=false` desactiva el uso de profundidad dentro de esos
estimadores de visión. Brain mantiene una suscripción directa a
`/boostercamera/head/depth` para su mapa de obstáculos; son rutas independientes.

## Cómo funciona la localización

El archivo `locator.h` declara explícitamente un **filtro de partículas**.
`Locator::locateRobot` recibe marcas L, T, X y P observadas en el marco del robot
y una caja de poses permitidas.

Proceso:

1. genera por defecto 200 hipótesis de pose;
2. transforma las marcas observadas al campo para cada hipótesis;
3. calcula el residual contra las marcas conocidas del mapa;
4. convierte los residuales en probabilidades;
5. remuestrea, reduce cantidad y rango de partículas;
6. repite hasta 20 iteraciones o convergencia;
7. hace un ajuste final y rechaza resultados de baja calidad.

Valores auditados:

| Parámetro | Valor |
|---|---:|
| Partículas iniciales | `200` |
| Offset inicial X/Y | `2.0 m` |
| Offset angular | `π/4` |
| Tolerancia de convergencia | `0.2` |
| Máximo de iteraciones | `20` |
| Reducción de partículas | `0.85` por iteración |
| Reducción de offsets | `0.8` por iteración |
| Mínimo de marcas en YAML | `4` |
| Residual máximo en YAML | `0.4` |

Códigos del localizador:

| Código | Resultado |
|---:|---|
| `0` | éxito |
| `1` | no quedan partículas |
| `2` | residual inválido |
| `3` | no convergió |
| `4` | muy pocas marcas |
| `5` | probabilidades demasiado bajas |

## Localización inicial paso a paso

### Preparación

1. ponga las dimensiones y líneas reglamentarias del campo;
2. coloque el T2 en una zona con al menos cuatro marcas distinguibles;
3. elimine personas/objetos que tapen líneas;
4. use iluminación semejante a la competencia;
5. compruebe RGB, pose de cabeza y detecciones;
6. sostenga el robot para la primera prueba de movimiento.

Comprobaciones:

```bash
ros2 topic hz /boostercamera/head/rgb
ros2 topic hz /boostercamera/head/head_pose_stamped
ros2 topic hz /booster_vision/detection
ros2 topic hz /booster_vision/line_segments  # si segmentación está activa
```

En el código de visión la pose preferida se llama `/head_pose_stamped`, mientras
en la unidad se observó también la ruta de cámara
`/boostercamera/head/head_pose_stamped`. Use `ros2 topic list | grep head_pose`
y confirme remapeo/bridge; un nombre equivocado impide proyectar marcas.

### Automática mediante GameController

1. inicie visión y brain con `game.xml`;
2. inicie GameController y manténgalo en `INITIAL`;
3. confirme que brain recibe `INITIAL` y `odom_calibrated` comienza en `false`;
4. la cabeza debe ejecutar `CamScanField`;
5. `SelfLocateEnterField` evalúa las marcas e intenta fijar la pose;
6. espere a que el log cambie a `Calibrated: YES`;
7. pase a `READY` solo cuando la pose sea coherente con la ubicación física;
8. en `READY` el robot puede caminar hacia la posición inicial del rol.

No es necesario levantar ni desplazar manualmente el robot si el campo visible
ofrece suficientes marcas. Puede ayudar cambiar su orientación con el robot
detenido, pero moverlo mientras se calcula una observación degrada la sincronía.

### Recalibración manual de localización

1. coloque el robot en la posición de entrada prevista por el árbol;
2. quite obstáculos y sosténgalo si la rama puede caminar;
3. pulse `LT+A` una sola vez;
4. confirme en el log `ControlState: 2` y `Calibrated: NO`;
5. deje que la cámara escanee;
6. espere una pose plausible y `Calibrated: YES`;
7. pulse `LT+B` para volver a `ACTION` solo cuando la zona sea segura.

`LT+A` no es una calibración inmóvil garantizada: el subárbol incluye
`RobocupWalk`/`SelfLocateEnterField` y puede ordenar movimiento.

## Cómo saber si ya está localizada

La fuente directa es `brain.log`. `logStatusToConsole` imprime periódicamente:

```text
FIELD_POSE:
    x: ... m    y: ... m    theta: ... rad (...)    Calibrated: YES
...
ControlState: 3
```

Véalo en vivo:

```bash
cd ~/T2_5v5Demo_Whrg
tail -F brain.log
```

Una pose válida debe cumplir las tres condiciones:

- `Calibrated: YES`;
- `x/y/theta` son físicamente plausibles y estables con el robot quieto;
- al mover el robot de forma controlada, la pose cambia con dirección y escala
  correctas y vuelve a corregirse al ver marcas.

Un booleano `YES` con pose reflejada, girada 180° o fuera del campo no es una
localización útil.

## Visualización en Rerun

El brain crea dos flujos llamados `robocup`: uno TCP y otro a archivo. La
configuración de esta rama es:

```yaml
rerunLog:
  enable_tcp: false
  server_ip: "10.0.16.110:9876"
  enable_file: true
  log_dir: "/home/booster/Workspace/rrlog"
```

### Opción A: archivo, la más fácil de validar

```bash
find /home/booster/Workspace/rrlog -type f -name '*.rrd' -printf '%TY-%Tm-%Td %TH:%TM:%TS %p\n' | sort
```

Copie el archivo a la PC:

```bash
scp booster@IP_DEL_ROBOT:/home/booster/Workspace/rrlog/RUTA/archivo.rrd .
```

Ábralo con una versión compatible de Rerun:

```bash
rerun archivo.rrd
```

El demo genera archivos dentro de una carpeta timestamped con sufijo de jugador
y equipo.

### Opción B: streaming por Wi-Fi

El código usa la API TCP antigua de Rerun 0.20.3 y espera `IP:PUERTO`. Para
evitar incompatibilidades, use Viewer 0.20.3 en la PC.

1. determine la IP Wi-Fi de la PC alcanzable desde el robot;
2. abra Rerun Viewer en la PC; en 0.20.x ejecutar `rerun` hace que escuche en
   `9876`;
3. permita TCP 9876 en el firewall para la red privada;
4. pruebe desde el robot `nc -vz IP_PC 9876`;
5. ponga `rerunLog.server_ip: "IP_PC:9876"` y `enable_tcp: true`;
6. reinicie brain;
7. compruebe en `brain.log` si falló `Connect rerunLog server`.

No use la IP Ethernet del robot como `server_ip`: el servidor es la PC que
muestra Rerun. Ethernet para SSH y Wi-Fi para Internet pueden coexistir, pero la
tabla de rutas y el firewall deben permitir robot → PC:9876.

### Entidades que debe añadir a los paneles

En el árbol de entidades busque:

```text
field/mapLines
field/robot
field/ball
field/teammate-*
field/identified_markings
field/detection_points
field/robots
image/img
image/detection_boxes
debug/*
field/goalkeeper/predicted_ball_trajectory
field/goalkeeper/intercept_point
```

Cree una vista 2D con `field/mapLines`, `field/robot`, `field/ball` y marcas.
La flecha/figura del robot debe moverse sobre el mapa. La trayectoria naranja y
el punto de intercepción rojo solo existen cuando el predictor produce datos
válidos.

Si Rerun abre vacío:

1. confirme que `enable_file` o `enable_tcp` es `true` en el proceso instalado;
2. confirme que el `.rrd` crece o que TCP 9876 está conectado;
3. expanda el árbol `field`/`image`; no todos los paneles se crean solos;
4. confirme que brain está vivo y tiene datos, no solo que Rerun está abierto;
5. use primero un `.rrd` local para separar problema de datos y problema de red.

## Calibración física de cámara

Esto es distinto de localizarse. El script disponible ejecuta:

```bash
cd ~/T2_5v5Demo_Whrg
./scripts/start_calibration.sh
```

Lanza `calibration_node handeye`, escribe el resultado temporal en
`/tmp/vision.yaml` y, si existe, lo copia con `sudo` a
`/opt/booster/vision.yaml`.

No ejecute este procedimiento como reacción automática a una localización que
falló. Primero determine si faltan líneas, pose de cabeza, configuración del
campo o sincronía. Una mala calibración física altera todas las proyecciones.
