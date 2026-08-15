# Intercepción adelantada adaptativa (2026-08-16)

## Objetivo

`GoalkeeperBlockShot` ya no tiene que esperar siempre sobre la línea defensiva.
Con `goalkeeper.prediction.intercept.enabled=true`, busca el punto más adelantado
de la trayectoria que todavía resulte alcanzable antes que el balón.

- Tiro lateral: ordena una diagonal hacia delante y hacia el punto de impacto.
- Tiro centrado: prioriza el avance frontal y satura `vx` con
  `front_vx_limit` para reducir el riesgo de que pase entre los pies.
- Si el cálculo conservador aún no encuentra un punto frontal alcanzable,
  ordena al menos `front_min_forward_distance` para no quedarse inmóvil.
- Ningún punto adelantado alcanzable: vuelve automáticamente a la línea fija.
- Estrategia desactivada: reproduce el bloqueo predictivo anterior.

## Cómo calcula el punto

1. Parte del cruce previsto con la línea defensiva.
2. Recorre candidatos desde `max_forward_distance` hacia atrás, separados por
   `search_step`.
3. Calcula cuándo llega el balón considerando su velocidad y
   `prediction.deceleration`.
4. Estima el tiempo del robot con la rapidez filtrada del odómetro multiplicada
   por `measured_speed_gain`, limitada entre `robot_speed_min` y
   `robot_speed_max`.
5. Elige el candidato más adelantado que satisface:

   `distancia_robot / rapidez_alcance + safety_time_sec <= tiempo_balon`

La adaptación ocurre en cada tick: si el robot medido va más lento, el objetivo
retrocede hacia la portería; si confirma más rapidez, puede adelantarlo.

## Valores locales/recomendados

| Parámetro | Valor | Al aumentarlo | Al disminuirlo |
|---|---:|---|---|
| `intercept.enabled` | `true` | activa la estrategia | recupera línea fija |
| `max_forward_distance` | `1.20 m` | diagonal más adelantada | bloqueo más profundo |
| `front_max_forward_distance` | `1.60 m` | sale antes ante tiro central | espera más atrás |
| `front_min_forward_distance` | `0.40 m` | salida frontal de emergencia más larga | salida mínima más corta |
| `front_lateral_threshold` | `0.25 m` | más tiros usan salida frontal | exige tiro más centrado |
| `min_ball_separation` | `0.25 m` | conserva más separación | permite cortar más cerca |
| `search_step` | `0.10 m` | menos precisión/cálculo | objetivo más fino |
| `robot_speed_min` | `0.45 m/s` | más optimista al arrancar | más conservador |
| `robot_speed_max` | `1.20 m/s` | permite mayor alcance | limita el adelanto |
| `measured_speed_gain` | `1.20` | confía más en odometría | retrocede el objetivo |
| `safety_time_sec` | `0.12 s` | más margen y menos adelanto | más agresivo/riesgoso |
| `diagonal_vx_limit` | `1.00 m/s` | diagonal más rápida | avance diagonal menor |
| `front_vx_limit` | `1.50 m/s` | salida frontal más rápida | salida frontal limitada |

## Filtro de continuidad

`goalkeeper.prediction.continuity_filter_enabled` controla dos protecciones:
rechazo temporal de candidatos que saltan más que `max_sample_jump` y reinicio
del historial ante ese salto. Desactivarlo permite reacquisición inmediata,
pero una falsa detección puede invertir el objetivo de bloqueo. No desactiva el
filtro exterior del campo.

## Cambios de aproximación solicitados

El perfil local/recomendado usa además:

```yaml
goalkeeper.chase.target_distance: 0.4
goalkeeper.chase.safe_distance: 0.6
goalkeeper.adjust.range: 0.3
goalkeeper.adjust.vy_limit: 0.2
obstacle_avoidance.avoid_during_chase: true
obstacle_avoidance.chase_ao_safe_dist: 1.5
```

`adjust.vx_limit` permanece en `1.5 m/s`; sólo se redujo la corrección lateral.

## Validación pendiente en robot

Este cambio se desarrolló y validó localmente sin conexión al T2. Antes de
competir hay que compilar en el robot, probar elevado el sentido de X/Y y luego
medir en suelo. La rapidez del odómetro es una medición, pero no garantiza que
la aceleración instantánea alcance el objetivo; ajustar primero
`safety_time_sec`, `robot_speed_max` y los dos límites de `vx`.
