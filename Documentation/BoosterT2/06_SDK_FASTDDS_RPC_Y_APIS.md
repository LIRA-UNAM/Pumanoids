# SDK T2, Fast DDS, RPC y APIs

## Capas disponibles

```text
Aplicación
├── B1LocoClient / clientes de cámara, visión, luz, AI y audio
├── RpcClient: petición/respuesta correlacionada por UUID
├── ChannelPublisher / ChannelSubscriber: topics tipados
└── Fast DDS: descubrimiento, transporte, QoS e interfaz de red
```

Para una aplicación nueva use primero los clientes de alto nivel. Use RPC crudo
solo cuando no exista wrapper y DDS directo para telemetría de alta frecuencia o
integración de bajo nivel.

## Compilar los ejemplos del SDK

```bash
cd ~/sdk_release
mkdir -p build
cd build
cmake ..
cmake --build . -j"$(nproc)"
```

El proyecto usa C++17, la biblioteca estática de la arquitectura, `Threads`,
`dl` y `rt`. El SDK Python se distribuye aparte:

```bash
python3 -m pip install booster_robotics_sdk_python --user
```

No asuma que la versión más reciente de PyPI coincide con el release C++ del
robot. Registre las dos versiones.

## Inicialización de red

La forma C++ es:

```cpp
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>

using booster::robot::ChannelFactory;
using booster::robot::b1::B1LocoClient;

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    ChannelFactory::Instance()->Init(0, argv[1]);

    B1LocoClient client;
    client.Init();
    if (!client.WaitForService(5000, true)) return 3;

    booster::robot::b1::GetRobotInfoResponse info;
    return client.GetRobotInfo(info);
}
```

El primer argumento de `Init` es el dominio DDS; el segundo selecciona la
interfaz de red. Los ejemplos lo llaman `networkInterface`. La implementación
precompilada no documenta aquí si espera nombre o IP en todos los releases: use
el formato que acepte el ejemplo del release instalado y evite una interfaz de
Internet que no tenga al T2.

También existen:

```cpp
Init(json_config);
InitDefault(domain_id);
InitWithConfigPath(domain_id, "fastdds.xml");
```

Todos los participantes que deben descubrirse tienen que compartir dominio,
interfaz alcanzable y políticas DDS compatibles.

## API locomotora de alto nivel

Servicio: `loco`. Versión observada en el cliente: `1.0.0.1`.

### Información y estado

| Método | ID | Resultado/uso |
|---|---:|---|
| `ChangeMode` | 2000 | Cambia Damping/Prepare/Walking/Custom/Soccer. |
| `GetMode` | 2017 | Modo actual. |
| `GetStatus` | 2018 | Modo, body control y acciones activas. |
| `GetRobotInfo` | 2022 | Nombre, versión, modelo, serie, edición y región. |
| `GetSensors` | 2044 | Catálogo de IMUs/sensores. |
| `GetHands` | 2045 | Catálogo de manos/end effectors. |
| `GetRobotModel` | 2046 | Modelo derivado de URDF, enlaces, joints y límites. |
| `GetTrainedTrajStatus` | 2047 | Estado e ID de trayectoria entrenada. |

### Movimiento, cabeza y fútbol

| Método | ID | Efecto |
|---|---:|---|
| `Move` / `MoveCommand` | 2001 | `vx`, `vy` m/s y `vyaw` rad/s; la segunda no espera respuesta. |
| `RotateHead` | 2004 | Pitch/yaw objetivo. |
| `RotateHeadWithDirection` | 2006 | Movimiento direccional limitado. |
| `RotateHeadWithTime` | 2043 | Pose objetivo y duración en ms. |
| `GetFrameTransform` | 2011 | Transformación entre cuerpo, cabeza, manos y pies. |
| `Shoot` | 2024 | Patada potente predefinida. |
| `VisualKick(start, version)` | 2038 | Inicia/detiene patada visual V1/V2. |
| `SwitchGait` | 2042 | Selecciona una de cuatro caminatas humanlike. |
| `EnterWBCGait` / `ExitWBCGait` | 2035/2036 | Entra/sale de marcha whole-body-control. |
| `ResetOdometry` | 2031 | Reinicia odometría; invalida referencias que dependan de ella. |

### Postura, recuperación y acciones

| Método | ID | Efecto |
|---|---:|---|
| `LieDown` | 2007 | Acostarse boca arriba; marcado inestable. |
| `GetUp` | 2008 | Levantado V1/V2. |
| `GetUpWithMode` | 2025 | Levantado terminando en Walking o Soccer. |
| `WaveHand` | 2005 | Abrir/cerrar durante gesto de saludo. |
| `Handshake` | 2015 | Inicia/cierra saludo de mano. |
| `HandOnChestGreeting` | 2050 | Mano al pecho/retorno. |
| `Dance` | 2016 | Danza/gesto por ID. |
| `WholeBodyDance` | 2029 | Danza de cuerpo completo. |
| `UpperBodyCustomControl` | 2030 | Activa/desactiva control custom superior. |
| `ZeroTorqueDrag` | 2026 | Arrastre/grabación por torque cero. |
| `RecordTrajectory` | 2027 | Inicia/detiene grabación. |
| `ReplayTrajectory` | 2028 | Reproduce archivo de trayectoria. |

### Manos y grippers

| Método | ID | Notas |
|---|---:|---|
| `MoveHandEndEffectorWithAux` | 2009 | Pose objetivo, punto auxiliar, tiempo y mano. |
| `MoveHandEndEffector` | 2009 | **Deprecado** por offset rotacional implícito. |
| `MoveHandEndEffectorV2` | 2009 | Sustituto recomendado. |
| `MoveDualHandEndEffector` | 2037 | Objetivos simultáneos de ambas manos. |
| `StopHandEndEffector` | 2023 | Detiene el movimiento de mano. |
| `SwitchHandEndEffectorControlMode` | 2012 | Habilita/deshabilita el controlador. |
| `ControlGripper` | 2010 | Posición/fuerza/velocidad. |
| `ControlDexterousHand` | 2013 | Parámetros por dedo y tipo de mano. |

Llame antes a `GetHands`; estos métodos no demuestran que la unidad tenga el
periférico.

### Trayectorias y lion dance

| Método | ID |
|---|---:|
| `LoadCustomTrainedTraj` | 2032 |
| `ActivateCustomTrainedTraj` | 2033 |
| `UnloadCustomTrainedTraj` | 2034 |
| `LionDancePrepare` | 2039 |
| `LionDanceStart` | 2040 |
| `LionDanceMove` | 2041 |

`LionDanceStart` está estrictamente permitido solo desde Prepare/Move. El propio
encabezado advierte que otra transición produce saltos articulares bruscos y
riesgo de daño. Trate todas las acciones entrenadas con una máquina de estados,
no como llamadas independientes.

### Sonido

`PlaySound` (2020) y `StopSound` (2021) controlan sonido desde el servicio
locomotor. El SDK también ofrece una API de audio separada para dispositivos,
volumen, mute, Bluetooth, reproducción, grabación, captura PCM y dirección de
llegada del sonido.

## APIs opcionales de otros servicios

| Cliente | Funciones visibles |
|---|---|
| `CameraClient` | `GetCameras` (3100). |
| `X5CameraClient` | cambiar modo normal/alta resolución y obtener estado (5001/5002). |
| `VisionClient` | iniciar (3000), detener (3001), obtener objetos (3002). |
| `HandEyeCalibClient` | iniciar, detener, estado, resultado y aplicar (3100–3104). |
| `LightControlClient` | color RGB, múltiples LEDs, detener control (2000–2002). |
| `AiClient` | chat, habla y tracking de cara (2000–2004). |
| `LuiClient` | ASR start/stop (1000/1001), TTS start/stop/texto (1050–1052). |

Los rangos de ID se repiten porque pertenecen a servicios RPC diferentes. No
envíe `api_id=2000` sin seleccionar el canal/servicio correcto.

## Fast DDS por topics

`ChannelFactory` crea canales tipados de envío/recepción. Por defecto
`reliable=false`; el QoS debe acordarse según pérdida tolerable y latencia.

```cpp
auto pub = std::make_shared<booster::robot::ChannelPublisher<Msg>>(topic, true);
pub->InitChannel();
if (pub->GetMatchedSubscriptionsCount() == 0) {
    // todavía no hay consumidor
}

booster::robot::ChannelSubscriberOptions options;
options.reliable = true;
auto sub = std::make_shared<booster::robot::ChannelSubscriber<Msg>>(
    topic, callback, options);
sub->InitChannel();
```

El subscriber expone métricas de cola/drop y número de publishers emparejados.
Eso debe monitorearse en aplicaciones de percepción de alta frecuencia.

Topics definidos por la capa B1:

```text
rt/joint_ctrl                    rt/low_state
rt/fall_down                     rt/odometer_state
rt/booster_hand_data             rt/booster_hand_touch_data
rt/tf                            rt/robot_states
rt/prone_body_control_status     rt/robot_replay_traj_id
rt/trained_traj_status           rt/robocup_behavior_status
rt/kick_ball                     rt/odom
rt/imu/data                      rt/joint_states
rt/X5CameraControl
```

## RPC sobre DDS

`RpcClient` usa `RpcReqMsg` y `RpcRespMsg`. El flujo es:

```text
WaitForService
  → generar UUID
  → publicar Request(api_id, body, versión)
  → servidor ejecuta
  → respuesta con el mismo UUID
  → desbloqueo o timeout
```

Valores por defecto del header auditado:

- espera de servicio: 5000 ms;
- respuesta de una API: 1000 ms;
- fire-and-forget: hasta 1000 ms para emparejar endpoint.

Use fire-and-forget solo para comandos repetitivos como velocidad, y acompañe
con una fuente independiente de estado. Para modo, patada, levantado,
calibración o trayectoria espere respuesta y verifique estado final.

## Relación con ROS 2 en el demo

El bridge Booster traduce la telemetría DDS a mensajes de `booster_interface`.
Brain no instancia directamente `B1LocoClient`; su `RobotClient` publica:

```text
LocoApiTopicReq  : booster_msgs/RpcReqMsg
LocoApiTopicResp : booster_msgs/RpcRespMsg
```

El body y header son JSON. La función de llamada del demo es asíncrona: que
`publish()` devuelva bien no significa que la acción haya terminado. La rama
actual agregó escucha de respuestas para casos críticos de recuperación, pero
una nueva función debe implementar explícitamente confirmación y timeout.

## Control de bajo nivel

El ejemplo `low_level_publisher` escribe `LowCmd` en `rt/joint_ctrl`. Antes de
adaptarlo:

1. descubra el modelo y mapa articular real;
2. asegure modo de control bajo nivel exclusivo;
3. detenga brain y cualquier controlador alto nivel;
4. use límites de posición, velocidad, torque y watchdog;
5. pruebe suspendido;
6. comience con ganancias conservadoras y un solo joint.

Nunca ejecute simultáneamente el demo RoboCup y un publisher articular de prueba.

## Patrón de aplicación robusta

```text
descubrir interfaces y versión
→ WaitForService
→ GetRobotInfo/GetRobotModel/GetStatus
→ validar estado permitido
→ enviar una acción acotada
→ verificar respuesta
→ observar estado/telemetría hasta condición final o timeout
→ mandar velocidad cero/cancelar ante error
→ registrar petición, respuesta y timestamps
```
