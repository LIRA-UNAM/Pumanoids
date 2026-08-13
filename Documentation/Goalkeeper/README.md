# Portero configurable — guía de operación

## Alcance

La rama `test_goalkeeper` se basa en `beijing_demo` y modifica únicamente el
demo ubicado en `beijing/`. El panel web controla parámetros reales del nodo
`/brain_node`; no es una simulación cuando se ejecuta en el robot.

## Flujo correcto para cambiar parámetros

1. Colocar el robot elevado, en `INITIAL` o en una zona despejada.
2. Abrir `http://IP_DEL_ROBOT:8088` y confirmar **Brain conectado**.
3. Pulsar **Recargar** para leer la configuración activa.
4. Modificar los parámetros deseados. Editar el formulario no envía cambios.
5. Pulsar **Aplicar en vivo** para una prueba temporal. Se pierde al reiniciar.
6. Tras validar el resultado, pulsar **Aplicar y guardar** para persistirlo en
   `beijing/beijing_ws/src/brain/config/config_local.yaml`.

Aplicar cualquiera de las dos opciones sí puede cambiar inmediatamente el
movimiento si el GameController permite actuar.

## Volver al comportamiento original

El archivo `factory_defaults.json` conserva un perfil protegido de 93 valores.
No se sobrescribe cuando la GUI guarda `config_local.yaml`.

1. Abrir **Ayuda y restauración**.
2. Pulsar **Cargar todos los valores originales** y confirmar.
3. Revisar el formulario; aún no se ha aplicado nada.
4. Pulsar **Aplicar y guardar**.

Los valores esenciales del demo original son:

```text
goalkeeper.mode=attack
goalkeeper.kick.type=default
goalkeeper.prediction.enabled=false
goalkeeper.prediction.require_localization=true
```

## Modos del portero

- `attack`: cubre la portería, pero puede perseguir, ajustar y despejar el
  balón cuando `GoalieDecide` determina que debe reclamarlo.
- `guard`: prioriza permanecer en cobertura. No ejecuta la secuencia normal de
  persecución y patada.

El modo original es `attack`.

## Selección de patada

En el grupo **Patada**, cambiar `goalkeeper.kick.type`:

- `default`: usa `Kick`, el despeje convencional del demo original.
- `visual`: usa `RLVisionKick` y permite elegir `kV1` o `kV2`.

Los parámetros de ambas patadas permanecen guardados. Sólo se ejecuta el tipo
seleccionado. Probar el cambio elevado y sin balón en rango antes de una prueba
en suelo.

## Predictor de tiros

El predictor se activa con `goalkeeper.prediction.enabled=true`. Acumula
detecciones del balón, ajusta una trayectoria ponderada, calcula velocidad,
calidad, desaceleración y cruce con la línea defensiva. Si el cruce amenaza la
portería dentro del horizonte configurado, `GoalieDecide` emite `block_shot` y
el robot se desplaza al punto lateral previsto.

Mantener `goalkeeper.prediction.require_localization=true`: sin
`odom_calibrated`, el historial se limpia y no se ordena el bloqueo.

Prueba inicial recomendada:

1. Robot elevado; predictor activo y localización lista.
2. Mover el balón hacia la portería y verificar aumento de muestras.
3. Confirmar `prediction_valid`, velocidad negativa en X y punto de cruce.
4. Mover el balón alejándose: no debe aparecer `block_shot`.
5. En suelo, comenzar con `goalkeeper.prediction.block.vy_limit=0.4` y una
   persona junto al paro de emergencia.

## Observabilidad

```bash
ros2 topic echo /brain/goalkeeper/decision
ros2 topic echo /brain/goalkeeper/status
```

Rerun recibe:

```text
field/goalkeeper/predicted_ball_trajectory
field/goalkeeper/intercept_point
```

El panel incluye un apéndice generado desde el esquema con los 93 parámetros,
valor original, rango, descripción y parte del robot afectada.

## Compilación y arranque

```bash
cd beijing
bash scripts/prepare_goalkeeper_build.sh
./scripts/start.sh role:=goal_keeper
```

Consultar también [IMPLEMENTATION.md](./IMPLEMENTATION.md) antes de transferir
la rama al robot.
