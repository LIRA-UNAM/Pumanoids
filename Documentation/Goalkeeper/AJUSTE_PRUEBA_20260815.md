# Ajuste de prueba del portero — sesión 2026-08-15 16:51

## Objetivo

Esta revisión corrige únicamente los dos problemas medidos en la sesión
`goalkeeper_telemetry_20260815_165144.jsonl`:

1. el sobrecontrol urgente tardó más en producir movimiento útil que el bloqueo
   normal;
2. la pelota seleccionada saltó repetidamente entre detecciones separadas.

No se añadieron nuevos modos de comportamiento ni parámetros experimentales.
El cambio de continuidad reutiliza `goalkeeper.prediction.max_sample_jump` y el
perfil desactiva la urgencia con `urgent_time_sec=0`.

## Evidencia que motivó el cambio

| Medición | Resultado |
|---|---:|
| Episodios `block_shot` | 23 |
| Tiempo mediano decisión → comando | 10.6 ms |
| Movimiento correcto, bloqueo normal | 85 ms |
| Movimiento correcto, bloqueo urgente | 345 ms |
| Tiempo disponible mediano al iniciar urgencia | 0.70 s |
| Rapidez física mediana durante urgencia | 0.318 m/s |
| Saltos de pelota mayores de 0.8 m | 410, 34.6/min de `PLAY` |
| Bloqueos con salto del cruce mayor de 0.5 m | 15 de 23 |
| Inversiones del comando lateral | 35 en 12 bloqueos |
| Goles recibidos marcados en GameController | 5 |

Los cinco goles entraron físicamente en la portería del robot. En el paquete del
GameController quedaron como `own_score`, porque se incrementó el equipo 5. Para
correlacionar automáticamente próximos fallos se debe incrementar el marcador
del equipo que anota, es decir, el contrario al portero.

## Cambios aplicados

### Perfil operativo estable

`beijing/beijing_ws/src/brain/config/config_local.yaml` y el botón **Cargar
perfil recomendado** usan ahora:

```yaml
history_msec: 600
max_samples: 20
recency_weight: 2.0
step_interval_msec: 100
step_count: 30
activation_hold_msec: 400
post_block_claim_msec: 2500
block.vx_limit: 0.65
block.vy_limit: 1.5
block.reaction_margin_sec: 0.10
block.urgent_time_sec: 0.0
```

Es el perfil inicial que tuvo el mejor comportamiento subjetivo, conservando
`alignment_tolerance=0.78`, el filtro exterior y la capacidad de despejar el
balón después del bloqueo.

Con `urgent_time_sec=0`, el código urgente permanece disponible pero no se
activa. `block_shot` utiliza el controlador normal y puede ordenar X e Y a la
vez; por eso vuelve a permitir el desplazamiento diagonal.

### Continuidad de la pelota

Mientras el predictor del rol `goal_keeper` está activo y la pelota anterior
tiene menos de 250 ms:

- se aceptan solamente candidatos a `max_sample_jump` metros o menos de la
  pelota anterior;
- los candidatos lejanos no reemplazan instantáneamente la trayectoria por
  tener una confianza ligeramente mayor;
- después de 250 ms sin una observación continua se recupera la selección
  original por máxima confianza.

El valor actual de `max_sample_jump` es `0.8 m`. El filtro no se aplica a
striker ni a otros roles y no sustituye el filtro exterior del campo.

La web y el JSONL añaden:

- `ball_jump_rejected_count`;
- `ball_jump_last_distance`;
- `ball_jump_last_x` y `ball_jump_last_y`.

### Rerun

`scripts/start.sh` crea `beijing/rrlog` y `config_local.yaml` configura
`rerunLog.log_dir: rrlog`. Esto evita el `Permission denied` observado en
`/home/booster/Workspace/rrlog`.

## Protocolo de la siguiente prueba

1. Compilar e instalar el workspace.
2. Colocar el robot vertical, campo despejado y parada de emergencia preparada.
3. Iniciar normalmente con GameController.
4. Abrir la web y pulsar **Recargar**. No es necesario cargar otro perfil si se
   está usando el `config_local.yaml` de esta rama.
5. Confirmar:

   ```bash
   ros2 param get /brain_node goalkeeper.prediction.history_msec
   ros2 param get /brain_node goalkeeper.prediction.max_samples
   ros2 param get /brain_node goalkeeper.prediction.block.urgent_time_sec
   ```

   Deben resultar `600.0`, `20` y `0.0`.

6. Realizar series separadas de tiros centrales, izquierda y derecha, anotando
   inmediatamente cada gol para el equipo contrario al portero.
7. No cambiar parámetros durante esta primera serie. Descargar el JSONL al
   terminar.

## Qué debe observarse

- `urgent_block` debe permanecer `false`.
- `decision_to_command_msec` debería permanecer cerca de 11 ms.
- `decision_to_aligned_motion_msec` debe acercarse al comportamiento normal
  anterior, alrededor de 85 ms y claramente por debajo de 345 ms.
- `ball_jump_rejected_count` puede aumentar cuando visión ofrece otra pelota o
  un falso positivo lejano.
- Deben reducirse las inversiones de `command_sent_y` dentro de un mismo
  `block_shot`.

Si una pelota real desaparece durante aproximadamente 250 ms antes de ser
recuperada, revisar primero el contador de saltos. El siguiente ajuste sería
subir `max_sample_jump` en pasos de `0.1 m`; no se debe cambiar simultáneamente
la ventana histórica ni la velocidad de bloqueo.

## Restauración del perfil anterior a esta prueba

Para comparar específicamente con la sesión 16:51, usar:

```text
history_msec=350
max_samples=12
recency_weight=2.5
step_interval_msec=50
step_count=60
activation_hold_msec=500
post_block_claim_msec=3000
block.vx_limit=0.7
block.reaction_margin_sec=0.25
block.urgent_time_sec=1.2
```

Los valores originales del demo siguen protegidos en `factory_defaults.json`.
