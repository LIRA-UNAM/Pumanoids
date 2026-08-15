# Perfil predeterminado 2026-08-14 20:24 y ajuste de patada

## Referencia exacta

El perfil operativo predeterminado reproduce la primera instantánea completa de
la sesión iniciada el 2026-08-14 a las 20:24. La instantánea fue registrada por
la web a las 20:27:39 UTC+8 como un evento `parameter_apply`.

No se modificó `factory_defaults.json`: “Restaurar fábrica” sigue recuperando
los valores originales del demo. “Aplicar perfil recomendado” recupera este
perfil de campo.

## Valores que distinguen el perfil

```yaml
goalkeeper:
  blocking:
    vy_limit: 0.9
    position_gain: 1.0
  chase:
    vx_limit: 0.5
    vy_limit: 1.5
    safe_distance: 0.5
  adjust:
    range: 1.0
    vx_limit: 1.5
    vy_limit: 1.5
  claim:
    max_ball_range: 3.0
    lateral_margin: 1.0
  kick:
    type: visual
    alignment_tolerance: 1.5707963268
    default:
      speed_limit: 0.5
      min_msec: 600
      stabilize_msec: 600
      exit_range: 3.0
      ball_move_threshold: 0.3
    visual:
      min_msec: 1500
      max_msec: 5000
      pre_delay_msec: 200
      post_delay_msec: 450
```

El predictor conserva el perfil de esa sesión: 600 ms de historia, 20 muestras
máximas, 5 mínimas, 100 ms de intervalo mínimo, bloqueo Y de 1.5 m/s y margen
de reacción de 0.10 s.

## Filtro de continuidad del balón

El filtro se aplica únicamente cuando el predictor está habilitado y el rol es
`goal_keeper`. Conserva como referencia el último balón aceptado durante una
ventana de 100 a 250 ms. En la configuración actual la ventana queda limitada
a 250 ms.

Durante esa ventana descarta un candidato si su posición de campo salta más de
`goalkeeper.prediction.max_sample_jump` (0.8 m) respecto del balón anterior.
Después de vencer la ventana vuelve a permitir una adquisición por confianza.

Su objetivo es evitar que dos detecciones etiquetadas como balón intercambien
el control del objetivo del portero en cuadros consecutivos. También reinicia
el historial del predictor cuando dos observaciones aceptadas se separan más
que el salto máximo.

Actualmente no existe un interruptor independiente para desactivarlo. Se
desactiva indirectamente al apagar `goalkeeper.prediction.enabled`, pero eso
también apaga todo el bloqueo predictivo. Aumentar `max_sample_jump` lo vuelve
más permisivo, pero no equivale a desactivarlo y la web limita el valor a 3 m.

## Parámetros relacionados con una patada fallida

### El balón pasa entre los pies

- `goalkeeper.kick.alignment_tolerance`: decide cuándo deja Adjust y comienza
  Kick. Menor valor obliga a alinearse mejor. El perfil histórico usa 1.5708
  rad; las pruebas posteriores indicaron mejor resultado cerca de 0.78 rad.
- `goalkeeper.adjust.range`: distancia radial buscada durante la alineación. Un
  valor demasiado grande puede autorizar la patada todavía lejos; demasiado
  pequeño puede hacer que el robot empuje o sobrepase el balón.
- `goalkeeper.adjust.turn_threshold`: un valor menor hace que comience antes la
  corrección angular.
- `goalkeeper.adjust.vx_limit` y `vy_limit`: valores excesivos pueden hacer que
  sobrepase el punto alineado antes de entrar a Kick.
- `goalkeeper.kick.default.enable_stabilize` y `stabilize_msec`: permiten una
  pausa/retiro pequeño antes de la patada convencional. Aumentan estabilidad,
  pero también aumentan el tiempo de despeje.

### El robot pasa lateralmente al lado del balón

- `goalkeeper.adjust.vy_limit`: limita el cruce lateral durante Adjust.
- `goalkeeper.adjust.range`: determina cuándo deja de acercarse radialmente.
- `goalkeeper.chase.target_distance`: distancia a la que Chase entrega el
  control a Adjust.
- `goalkeeper.chase.safe_distance`: modifica la separación usada con evasión
  de obstáculos; un valor grande puede desviar la aproximación.
- `obstacle_avoidance.avoid_during_chase`: puede producir una trayectoria
  lateral alrededor de una detección considerada obstáculo.
- `obstacle_avoidance.chase_ao_safe_dist`: controla cuánto rodea el obstáculo.

### La patada empieza pero no desplaza el balón

- `goalkeeper.kick.type`: selecciona `default` o `visual`; son controladores
  diferentes y deben evaluarse por separado.
- `goalkeeper.kick.default.speed_limit`: velocidad del crab-walk convencional.
- `goalkeeper.kick.default.min_msec`: duración mínima del movimiento.
- `goalkeeper.kick.default.abort_when_ball_moved`: termina el nodo al confirmar
  movimiento del balón o pérdida prolongada.
- `goalkeeper.kick.default.ball_move_threshold`: desplazamiento necesario para
  considerar confirmada la patada.
- `goalkeeper.kick.default.exit_range`: distancia a partir de la que termina la
  acción convencional.
- Para VisualKick: `visual.min_msec`, `visual.max_msec`, `visual.range`,
  `visual.pre_delay_msec` y `visual.post_delay_msec`.

## Orden recomendado de ajuste

1. fijar un solo tipo de patada;
2. ajustar `alignment_tolerance`;
3. ajustar `adjust.range` y reducir `adjust.vy_limit` si sobrepasa el balón;
4. ajustar duración/velocidad de la patada elegida;
5. modificar Chase solamente si el fallo ocurre antes de entrar a Adjust.

No se deben cambiar varios grupos en la misma serie de tiros: impide atribuir
el resultado a un parámetro concreto.
