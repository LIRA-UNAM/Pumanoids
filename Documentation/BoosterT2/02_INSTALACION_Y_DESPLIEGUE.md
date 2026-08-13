# Instalación, compilación y despliegue

## Estructura correcta

La raíz ejecutable del demo es la carpeta que contiene directamente:

```text
T2_5v5Demo_Whrg/
├── src/
├── scripts/
├── third_party/
├── configs/
├── build/       # generado por colcon
├── install/     # generado por colcon
└── log/         # generado por colcon
```

No agregue otra capa de workspace alrededor. Los scripts calculan rutas desde
su propia ubicación y esperan esta forma plana.

La rama de desarrollo usa `beijing/beijing_ws/src` como staging, mientras el
release desplegado espera `T2_5v5Demo_Whrg/src`. Consulte la advertencia de
[layouts y despliegue](08_DIAGNOSTICO_CODIGO_Y_FUENTES.md#dos-layouts-que-no-deben-confundirse)
antes de copiar esta rama al robot.

## Requisitos confirmados

- Ubuntu 22.04 o 24.04, arquitectura ARM64 en el T2;
- ROS 2 **Kilted** en `/opt/ros/kilted`;
- SDK Booster T2 instalado en `/usr/local`;
- CMake, compilador C++, colcon y rosdep;
- BehaviorTree.CPP, Backward ROS, Ceres, FFTW3 y `espeak`;
- Rerun C++ SDK 0.20.3 y Apache Arrow 18 instalados por el demo.

El README del SDK declara binarios precompilados para Ubuntu 22.04 con GCC
11.4, mientras su instalador también acepta Ubuntu 24.04 y selecciona la
biblioteca por arquitectura/versión. No copie una biblioteca `x86_64` al T2
ARM64.

## Instalación desde cero

### 1. Copiar las dos carpetas

Ejemplo de destinos limpios:

```text
/home/booster/sdk_release
/home/booster/T2_5v5Demo_Whrg
```

Compruebe que los archivos grandes de `third_party` no sean punteros de Git
LFS:

```bash
cd ~/T2_5v5Demo_Whrg
head -n 2 third_party/rerun_cpp_sdk-0.20.3-multiplatform.zip
head -n 2 third_party/apache-arrow-18.0.0.tar.gz
```

Si aparece `version https://git-lfs.github.com/spec/v1`, ejecute `git lfs pull`
desde un clon que tenga acceso al remoto antes de instalar.

### 2. Instalar el SDK T2

```bash
cd ~/sdk_release
sudo ./install.sh
```

El script instala encabezados en `/usr/local/include`, bibliotecas de la
arquitectura en `/usr/local/lib` y ejecuta `ldconfig`. Verificación:

```bash
find /usr/local/include -maxdepth 2 -iname '*booster*' | head
find /usr/local/lib -maxdepth 1 -iname '*booster*' -o -iname '*fastdds*'
ldconfig -p | grep -i booster
```

No mezcle dos releases del SDK en `/usr/local`. Registre el commit o número de
release que se copió.

### 3. Instalar dependencias del demo

```bash
cd ~/T2_5v5Demo_Whrg
bash scripts/install.sh
```

Este instalador:

1. exige `/opt/ros/kilted/setup.bash`;
2. instala paquetes del sistema y ROS;
3. instala Rerun C++ 0.20.3 bajo `/opt/rerun_sdk`;
4. actualiza rosdep, usando por defecto el espejo Tsinghua;
5. resuelve dependencias de todos los paquetes de `src`.

En una red distinta se pueden definir `ROSDEP_MIRROR` o
`ROSDISTRO_INDEX_URL`. `ROSDEP_SKIP=1` solo es aceptable cuando las dependencias
ya están instaladas; no convierte una instalación incompleta en válida.

### 4. Compilar

```bash
cd ~/T2_5v5Demo_Whrg
source /opt/ros/kilted/setup.bash
colcon build --symlink-install \
  --parallel-workers "$(nproc)" \
  --cmake-clean-cache
source install/setup.bash
```

El árbol actual contiene siete paquetes ROS:

```text
booster_msgs
booster_interface
brain
game_controller
game_controller_interface
vision_interface
vision
```

Compruebe los ejecutables:

```bash
ros2 pkg executables brain
ros2 pkg executables vision
ros2 pkg executables game_controller
ros2 pkg prefix brain
```

El prefijo debe resolver a `~/T2_5v5Demo_Whrg/install`, no a un workspace viejo.

## Configuración previa al arranque

Edite y revise estos archivos:

- `src/brain/config/config.yaml`;
- `src/game_controller/launch/launch.py`;
- `src/vision/config/vision.yaml`;
- `configs/fastdds.xml` o el perfil Booster instalado en `/opt/booster`.

Variables mínimas:

```yaml
game:
  team_id: 5
  player_id: 1
  player_role: goal_keeper

game_control_ip: 10.0.16.150
```

El receptor de GameController tiene además `ip_white_list`. El emisor debe
estar permitido allí; cambiar solo `game_control_ip` no arregla un paquete UDP
descartado por la lista blanca.

## Primera puesta en marcha, componente por componente

Ponga el robot en soporte, abra tres terminales y no habilite aún un estado que
ordene caminar.

Terminal 1 — visión:

```bash
cd ~/T2_5v5Demo_Whrg
source /opt/ros/kilted/setup.bash
source install/setup.bash
./scripts/start_vision.sh
```

Terminal 2 — brain:

```bash
cd ~/T2_5v5Demo_Whrg
source /opt/ros/kilted/setup.bash
source install/setup.bash
./scripts/start_brain.sh
```

Terminal 3 — GameController:

```bash
cd ~/T2_5v5Demo_Whrg
source /opt/ros/kilted/setup.bash
source install/setup.bash
./scripts/start_game_controller.sh
```

En un cuarto terminal:

```bash
ros2 node list
ros2 topic list | sort
ros2 topic hz /boostercamera/head/rgb
ros2 topic hz /booster_vision/detection
ros2 topic echo /robocup/game_controller --once
```

Solo después de confirmar cada entrada pruebe el lanzador total.

## Qué hace realmente `start.sh`

Además de iniciar visión, brain, GameController y la GUI del portero, el script
actual:

- mata `booster-video-stream`;
- ejecuta `stop.sh`;
- activa `jetson_clocks`;
- enmascara timers de APT y `unattended-upgrades`;
- borra la marca temporal de APT;
- mata `update_manager` y **todos los procesos `python3`**;
- deshabilita servicios Booster y de asistencia;
- fuerza un perfil Fast DDS UDP-only;
- inicia nodos con `nohup` y escribe logs en la raíz.

Por eso debe ejecutarse solo en una imagen de robot dedicada y tras revisar qué
otros procesos Python/servicios son necesarios. El efecto sobre servicios puede
persistir después de detener el demo.

Arranque total:

```bash
cd ~/T2_5v5Demo_Whrg
./scripts/start.sh role:=goal_keeper
tail -F vision.log brain.log game_controller.log goalkeeper_web.log
```

Parada del demo:

```bash
cd ~/T2_5v5Demo_Whrg
./scripts/stop.sh
```

`stop.sh` mata los procesos del demo y detiene un posible servicio dueño de UDP
3838. No restaura automáticamente los timers o servicios enmascarados por
`start.sh`.

## Compilación de cambios

Compilación completa:

```bash
cd ~/T2_5v5Demo_Whrg
source /opt/ros/kilted/setup.bash
./scripts/build.sh
```

Solo brain y dependencias:

```bash
./scripts/build_brain.sh
```

Después de modificar XML, YAML, launch o código, compruebe qué copia está usando
el proceso:

```bash
ros2 pkg prefix brain
readlink -f install/brain/share/brain
```

Sin `--symlink-install`, algunos archivos instalados son copias y requieren
recompilar para reflejar la modificación.

## Actualización controlada

1. guarde una copia del `config.yaml` operativo y del perfil de calibración;
2. registre `git rev-parse HEAD` del demo y del SDK;
3. compile sin robot en movimiento;
4. ejecute pruebas y revise warnings;
5. pruebe visión y comunicaciones en soporte;
6. pruebe una sola acción locomotora;
7. recién entonces haga una prueba de campo.

No ejecute instaladores de distribución que eliminen rutas de trabajo sin leerlos.
En particular, cualquier script que borre un workspace debe reemplazarse por un
despliegue a una carpeta nueva y un cambio explícito de versión.
