# Hardware, interfaces y variantes T2

## Qué se puede afirmar de la unidad auditada

El demo configura un robot de altura cinemática `1.28146 m` y cámara de cabeza
tipo D-Robotics. El robot inspeccionado ejecuta Ubuntu 24.04 en ARM64, ROS 2
Kilted y el SDK nativo instalado bajo `/usr/local`.

Interfaces observadas en el T2:

- Ethernet con dirección de laboratorio `192.168.10.102`;
- Wi-Fi en otra subred, que puede usarse para Rerun si existe ruta de ida y
  vuelta;
- cámara RGB de cabeza;
- cámara de profundidad y `camera_info`;
- pose de cabeza;
- IMU, odometría y estados articulares a través de la capa Booster;
- mando remoto publicado como estado normalizado de botones y ejes.

Los nombres de topics observados para la cámara son:

```text
/boostercamera/head/rgb
/boostercamera/head/depth
/boostercamera/head/rgb/camera_info
/boostercamera/head/head_pose_stamped
```

No se debe codificar la IP del robot en la lógica del demo. La dirección cambia
con la red; úsela solo para SSH, copiar logs o abrir la GUI.

## Variantes: qué significa realmente

El material local no trae un catálogo comercial fiable de submodelos T2. El
SDK de la rama T2 sí contempla distintas configuraciones de articulaciones y
periféricos, pero eso no garantiza que todas estén instaladas en cada T2.

Los mapas articulares expuestos por el SDK incluyen:

| Mapa del SDK | Articulaciones | Diferencia visible |
|---|---:|---|
| `JointIndex` | 23 | Cabeza, brazos de 4 GDL, cintura y piernas de 6 GDL. |
| `JointIndexK1` | 22 | Variante sin la articulación de cintura. No asumir que es T2. |
| `JointIndexWith7DofArm` | 29 | Brazos de 7 GDL y mapa ampliado. |

Estos son mapas compatibles con el SDK, no una lista de SKUs Booster T2. Antes
de mandar posiciones articulares hay que consultar el modelo real. Seleccionar
un índice por intuición puede accionar la articulación equivocada.

## Descubrimiento de la unidad en tiempo de ejecución

Las APIs de alto nivel son la fuente adecuada para identificar la unidad:

| Consulta | Qué devuelve | Uso |
|---|---|---|
| `GetRobotInfo` | Información/edición reportada por el servicio locomotor | Registrar modelo y versión antes de probar. |
| `GetRobotModel` | Enlaces, articulaciones y límites | Construir el mapa articular real. |
| `GetSensors` | Sensores anunciados | No depender de sensores ausentes. |
| `GetHands` | Manos disponibles | Elegir gripper/mano diestra solo si existe. |
| `CameraClient::GetCameras` | Cámaras anunciadas | Seleccionar cabeza/RGB/profundidad por capacidad. |

Flujo recomendado:

```text
inicializar Fast DDS con la interfaz de red
          ↓
esperar el servicio "loco"
          ↓
GetRobotInfo + GetRobotModel
          ↓
validar número/nombre/límites de joints
          ↓
GetSensors + GetHands + GetCameras
          ↓
habilitar únicamente funciones presentes
```

Guarde el resultado de descubrimiento junto con cada sesión de pruebas. Eso
permite explicar por qué el mismo binario se comporta distinto en dos unidades.

## Cinemática y marcos

El SDK ofrece `GetFrameTransform` y el modelo contiene el enlace raíz, enlaces,
articulaciones y límites. El demo mantiene, como mínimo, estas nociones:

- marco de campo, usado por localización, estrategia y Rerun;
- base del robot, usada para detecciones y movimiento relativo;
- cabeza/cámara, transformada hacia la base mediante la pose de cabeza;
- odometría, usada entre correcciones absolutas del campo.

No mezcle coordenadas de campo con coordenadas del robot. En el campo, `x` y
`y` describen una posición absoluta; en un comando de marcha, `vx`, `vy` y
`vyaw` son velocidades relativas al robot.

## Cámara usada por el demo

La configuración local define una imagen de `544 × 448` y campos de visión
aproximados de `105°` horizontal y `94°` vertical. Son parámetros del perfil de
este demo, no una especificación universal de toda cámara T2.

La cadena requiere sincronía entre:

1. RGB para detector y segmentación;
2. profundidad para proyectar detecciones;
3. `camera_info` para la proyección;
4. pose de cabeza para transformar al marco de base/campo.

Si uno de esos cuatro elementos falta o tiene timestamps incoherentes, puede
haber cajas correctas en la imagen pero posiciones de balón/campo erróneas.

## Interfaces de movimiento

Hay dos niveles que no deben competir:

- **alto nivel:** servicio `loco`, `B1LocoClient`, movimientos, cabeza, patada,
  levantado, gait y acciones;
- **bajo nivel:** `LowCmd`/control articular en `rt/joint_ctrl`.

El demo RoboCup usa principalmente el alto nivel, aunque su `RobotClient`
publica las peticiones RPC sobre ROS 2. El ejemplo de bajo nivel del SDK mueve
articulaciones directamente y debe considerarse una prueba de banco, no un
método normal de operar el demo.

## Periféricos opcionales visibles en el SDK

El SDK T2 auditado contiene clientes para cámaras, visión, luces, manos,
grippers, audio, ASR/TTS, conversación y seguimiento de cara. Su presencia en
los encabezados significa “API potencial”, no “hardware confirmado”.

La aplicación debe degradarse de forma explícita:

- si no hay manos, ocultar controles de mano;
- si no hay profundidad, no declarar distancia 3D confiable;
- si no hay servicio de audio, no bloquear el árbol por una locución;
- si no hay cámara anunciada, detener visión en vez de mover a ciegas.

## Inventario que falta confirmar en cada robot

Antes de documentar una unidad concreta como “lista para competencia”, anote:

- edición y firmware reportados;
- mapa y límites articulares;
- presencia de manos/grippers;
- cámaras y resoluciones anunciadas;
- interfaces Ethernet/Wi-Fi persistentes;
- versión del SDK instalada;
- versión de ROS y del demo;
- modo inicial del controlador locomotor;
- mecanismo físico de parada disponible en esa unidad.
