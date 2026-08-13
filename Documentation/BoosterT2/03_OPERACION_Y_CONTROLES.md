# Operación, estados y control remoto

## Lista de seguridad antes de energizar movimiento

- batería/cableado en condición normal;
- robot en soporte para la primera acción;
- suelo despejado y sin personas en la trayectoria;
- operador junto al robot y segundo operador en la computadora;
- `brain.log` y estado de GameController visibles;
- parada por software probada;
- equipo, jugador y rol correctos;
- no hay dos programas mandando movimiento al mismo tiempo.

`kDamping` elimina el esfuerzo de soporte: el robot puede caer inmediatamente.
No se debe usar como “parada de emergencia” si el robot no está sostenido.

## Estados del controlador locomotor

El SDK define estos modos generales:

| Modo | Valor | Significado operativo |
|---|---:|---|
| `kDamping` | 0 | Articulaciones amortiguadas/sin sostén normal; riesgo de caída. |
| `kPrepare` | 1 | Preparación para acciones y transición controlada. |
| `kWalking` | 2 | Locomoción. |
| `kCustom` | 3 | Acción o controlador personalizado. |
| `kSoccer` | 4 | Capacidades de fútbol. |

El `control_state` interno del brain es distinto: `1=CANCEL`, `2=RECALIBRATE`,
`3=ACTION`. No confunda el estado del árbol con el modo del SDK.

## Estados RoboCup

Con el árbol `game.xml`, el comportamiento esperado es:

| Estado | Qué hace el brain |
|---|---|
| `INITIAL` | Detiene desplazamiento, escanea/localiza al entrar y prepara percepción. |
| `READY` | Mueve cabeza, intenta ir a la posición inicial del rol y sigue localizando. |
| `SET` | Sigue percepción/localización, pero ordena velocidad cero. |
| `PLAY` | Ejecuta el subárbol del rol (`striker` o `goal_keeper`). |
| `END` | Detiene el robot. |

Además, penalización, timeout, fase de tiro libre y otros campos pueden forzar
parada aunque la interfaz del GameController diga “START”. Mire el mensaje ROS
completo, no solo el botón de la GUI.

El árbol `game.xml` ejecuta una acción inicial que coloca `control_state=3`. No
debería ser obligatorio pulsar `LT+B` para que funcione. Si lo es, busque árbol
equivocado, deriva del joystick, estado no recibido o un valor sobrescrito.

## Mando remoto reconocido

El mensaje `RemoteControllerState` contiene:

- sticks `lx`, `ly`, `rx`, `ry`;
- botones `A`, `B`, `X`, `Y`;
- `LB`, `RB`, `LT`, `RT`;
- `LS`, `RS`, `back`, `start`;
- cruceta arriba/abajo/izquierda/derecha.

### Combinaciones implementadas por brain

| Entrada | Acción real |
|---|---|
| `LT+X` | `CANCEL`: velocidad cero y cabeza a posición neutra. |
| `LT+A` | `RECALIBRATE`: invalida `odom_calibrated` y entra al flujo de calibración/localización. |
| `LT+B` | `ACTION`: habilita la rama automática. |
| `LT+Y` | Alterna en tiempo de ejecución `striker` / `goal_keeper`. |
| `LT+↑` / `LT+↓` | Aumenta/disminuye `vxFactor` en 0.01. |
| `LT+←` / `LT+→` | Cambia `yawOffset` en 0.01. |
| mantener `LB` | Activa `assist_chase`; al soltarlo se desactiva. |
| mantener `RB` | Activa `assist_kick`; al soltarlo se desactiva. |
| cruceta sin trigger | Sonidos de celebrar/arrepentirse/listo/provocar. |

Las combinaciones con `RT` no tienen una acción funcional implementada en el
callback auditado.

### Qué hace exactamente `LT+A`

No calibra instantáneamente ni debe iniciar una marcha abierta. Hace dos cosas:

1. cambia el estado interno a `RECALIBRATE`;
2. marca la odometría/localización como no calibrada.

El árbol activo reacciona ejecutando escaneo de cámara, detección de marcas y
`SelfLocateEnterField`. Según las marcas observadas y el árbol, puede ordenar
rotación o reposicionamiento. Por eso el robot puede empezar a moverse “solo”.
No existe en esa combinación una garantía geométrica de que no alcance el borde
o un obstáculo: la seguridad depende de percepción, configuración del campo y
supervisión.

Si no quiere recalibrar, no pulse `LT+A`. La calibración manual de cámara es un
procedimiento distinto y no debe ejecutarse durante un partido.

### Sticks y toma manual

Si cualquier stick supera aproximadamente `0.1`, el brain pone `go_manual=true`
y suprime la rama automática. Ese callback no convierte por sí mismo los ejes
en velocidades; funciona como compuerta de toma manual.

Consecuencia: un stick con deriva puede dejar uno o ambos robots inmóviles. En
reposo, observe:

```bash
ros2 topic echo /remote_controller_state
```

Los cuatro ejes deben permanecer cerca de cero. Aumentar el umbral puede ocultar
el síntoma, pero lo correcto es calibrar/reparar el mando.

`scripts/start_joystick.sh` inicia el `joy_node` estándar, que normalmente
publica `/joy`; brain escucha `/remote_controller_state`. Sin un puente o el
driver Booster que produzca ese mensaje, ejecutar ese script no conecta un
gamepad genérico al brain.

## Cuatro tipos de caminata del SDK

`SwitchGait` expone cuatro tipos:

1. `whole body humanlike`;
2. `half body humanlike`;
3. `half body humanlike V2`;
4. `whole body humanlike V2`.

“Whole body” incorpora más movimiento coordinado del torso/brazos; “half body”
prioriza la parte inferior. Las versiones V2 son revisiones del controlador.
No son lo mismo que `Walking`, `Soccer`, `VisualKick` o `WBCGait`, que son modos
o acciones diferentes.

Para activarlas desde ingeniería se usa `B1LocoClient::SwitchGait` después de
esperar el servicio locomotor y verificar un estado compatible. El demo puede
cambiar de controlador al ejecutar acciones de fútbol; no altere gait durante
una patada, levantado o bloqueo.

## Arranque operativo por fases

1. inicie cámara/visión y confirme frecuencia;
2. inicie brain con robot en soporte;
3. confirme `FIELD_POSE`, estado y `Calibrated` en `brain.log`;
4. inicie GameController y verifique `/robocup/game_controller`;
5. pase por `INITIAL`, `READY` y `SET` observando cada transición;
6. en `READY`, mantenga espacio para el trayecto hacia la posición del rol;
7. entre a `PLAY` solo con campo despejado;
8. mantenga `LT+X`, `stop.sh` y el medio físico de parada preparados.

## Qué hacer si empieza a moverse sin esperarlo

1. pulse `LT+X` si el topic del mando está confirmado;
2. ejecute `./scripts/stop.sh` desde una sesión ya abierta;
3. use la parada física segura de la unidad si software no responde;
4. sostenga el robot antes de cualquier cambio a damping;
5. no reinicie hasta identificar estado GC, árbol, control state y último
   comando locomotor.

`LT+X` depende de ROS, brain y el topic del mando; no reemplaza una parada
física independiente.
