# Navegación Social e Interacción (Social Navigation)

Este paquete contiene la máquina de estados principal (`greet_and_return_sm`) encargada de orquestar el comportamiento social del robot Eddi. Conecta la visión artificial, el control de movimiento físico y un agente de Inteligencia Artificial conversacional para crear una interacción humano-robot autónoma y fluida.

---

## 🧠 Arquitectura y Nodos Vinculados

El sistema funciona a través de la cooperación de múltiples módulos repartidos en diferentes paquetes del workspace:

### 1. Control Central (Este paquete: `social_navigation`)
* **`greet_and_return_sm`**: Es el director de orquesta. Funciona mediante una máquina de 4 estados:
  * **IDLE (Buscando):** El robot está quieto de la base, pero el cuello escanea el entorno. Si detecta un rostro fresco, activa el seguimiento y pasa a *APPROACHING*.
  * **APPROACHING (Acercándose):** El robot navega hacia la persona. Si llega a 1 metro de distancia, se detiene y pasa a *GREETING*. Si la persona se pierde de vista por más de 10 seg, regresa a *IDLE*.
  * **GREETING (Saludando):** Hace un llamado al servicio RPC (`/booster_rpc_service`) para levantar la mano físicamente. Espera 5 segundos y pasa a *INTERACTING*.
  * **INTERACTING (Conversando):** Activa el tópico `/interaction/enable` despertando al Agente de IA. Ignora los rostros que estén a más de 66 cm. Si la persona desaparece por más de 15 segundos, apaga la IA y regresa a *IDLE*.

### 2. Agente de Inteligencia Artificial (El Cerebro, Oído y Boca)
* **`ollama_planning_node` (Paquete: `llm_planning`)**: El "Cerebro". Se conecta a un servidor local de **vLLM**. Contiene el System Prompt que le da a Eddi su personalidad de RRHH. Se activa/desactiva según la señal de la máquina de estados.
* **`faster_whisper_node` (Paquete: `speech2text`)**: El "Oído". Escucha el micrófono continuamente mediante Voice Activity Detection (VAD) y traduce el audio a texto usando Whisper optimizado.
* **`text_to_speech_subscriber` (Paquete: `text2speech`)**: La "Boca". Utiliza **PiperTTS** (modelo ONNX) para sintetizar el texto de la IA y reproducirlo por la bocina del robot.

### 3. Visión y Seguimiento (Los Ojos, Cuello y Base)
* **`face_detector` (Paquete: `face_detector`)**: Usa MediaPipe para encontrar rostros en la imagen de la cámara, estima la distancia y publica coordenadas en `/vision/face`.
* **`deepface_follower_node` (Paquete: `person_fallower`)**: Controla el cuello del robot (pan/tilt). Si no hay rostros, realiza un patrón de escaneo de búsqueda. Si hay rostros, los centra en la imagen. No se desactiva nunca.
* **`person_fallower` (Paquete: `person_fallower`)**: Controla las llantas de la base. Recibe el rostro y genera velocidades en `/cmd_vel` para acercarse a la persona. Respeta la señal `/person_follower/enable` para frenar por completo durante la plática.

---

## ⚙️ Dependencias e Instalación

Para asegurar que todos los nodos funcionen, verifica tener instaladas las siguientes dependencias:

### 1. Dependencias de Sistema (Ubuntu/Linux)
Requeridas por PyAudio y Piper para procesar y reproducir sonido:
```bash
sudo apt-get update
sudo apt-get install portaudio19-dev alsa-utils
```

### 2. Dependencias de Python
Instala las librerías de Inteligencia artificial, audio y visión:
```bash
pip install faster-whisper pyaudio numpy openai piper-tts mediapipe opencv-python
```

### 3. Permisos de Docker
Para que vLLM se ejecute automáticamente desde el launch sin pedir contraseña de `sudo`:
```bash
sudo usermod -aG docker $USER
newgrp docker
```

---

## 🚀 Archivos Launch

El sistema se compone de varios archivos Launch para orquestar los diferentes módulos:

### 1. Launch Maestro: `social_interaction.launch.py` (Paquete: `social_navigation`)
**Propósito:** Es el punto de entrada principal para el comportamiento del robot. Orquesta la interacción de alto nivel.
* **Nodos que ejecuta directamente:** `greet_and_return_sm` (Máquina de estados principal).
* **Launchs anidados que incluye:** Manda a llamar a `agente_SD.launch.py` para levantar la IA como un sub-módulo.
```bash
ros2 launch social_navigation social_interaction.launch.py
```
*(Nota: Este launch asume que la percepción visual y los motores ya están encendidos en un proceso paralelo).*

### 2. Launch Secundario (Inteligencia Artificial): `agente_SD.launch.py` (Paquete: `llm_planning`)
**Propósito:** Es invocado internamente por el Launch Maestro, pero también puede ejecutarse solo para pruebas. Levanta todo el stack conversacional de forma simultánea.
* **Procesos que ejecuta internamente:**
  1. **vLLM (Docker `ExecuteProcess`):** Arranca el contenedor de Nvidia y carga el modelo local de IA (`Qwen3`) guardando la caché en la Jetson.
  2. **`faster_whisper_node`:** Inicia el micrófono para escuchar y transcribir voz a texto (ASR).
  3. **`ollama_planning_node`:** El "cerebro". Lee el System Prompt de RRHH y genera la respuesta conversacional.
  4. **`text_to_speech_subscriber`:** Lanza PiperTTS (La "Boca") para convertir el texto de la IA en audio por las bocinas.
```bash
ros2 launch llm_planning agente_SD.launch.py
```

### 3. Launch de Percepción y Movimiento: `person_fallower.launch.py` (Paquete: `person_fallower`)
**Propósito:** Agrupa y levanta todos los nodos encargados de los "Ojos, Cuello y Piernas". Es la capa de hardware y reacción física que permite al robot ver y acercarse a las personas.
* **Procesos que ejecuta internamente:**
  1. **Launch anidado `t1_twist.launch.py`:** Levanta el driver de control de los motores de la base.
  2. **`face_detector`:** Inicia la detección de rostros procesando las imágenes de la cámara (`/camera/color/image_raw`).
  3. **`deepface_follower_node`:** Inicia el patrón de búsqueda constante y el control de los motores del cuello.
  4. **`person_fallower`:** Inicia el cálculo matemático de velocidades (`/cmd_vel`) para acercar la base del robot a la persona detectada.
```bash
ros2 launch person_fallower person_fallower.launch.py
```
*(Nota: Por seguridad, el seguimiento de la base arranca desactivado esperando la orden de la máquina de estados. Para que las llantas comiencen a moverse en una prueba independiente, debes habilitarlo desde otra terminal:)*
```bash
ros2 topic pub -1 /person_follower/enable std_msgs/msg/Bool "{data: true}"
```

### 3. Launch de Percepción y Movimiento: `person_fallower.launch.py` (Paquete: `person_fallower`)
**Propósito:** Agrupa y levanta todos los nodos encargados de los "Ojos, Cuello y Piernas". Es la capa de hardware y reacción física que permite al robot ver y acercarse a las personas.
* **Procesos que ejecuta internamente:**
  1. **Launch anidado `t1_twist.launch.py`:** Levanta el driver de control de los motores de la base.
  2. **`face_detector`:** Inicia la detección de rostros procesando las imágenes de la cámara (`/camera/color/image_raw`).
  3. **`deepface_follower_node`:** Inicia el patrón de búsqueda constante y el control de los motores del cuello.
  4. **`person_fallower`:** Inicia el cálculo matemático de velocidades (`/cmd_vel`) para acercar la base del robot a la persona detectada.
```bash
ros2 launch person_fallower person_fallower.launch.py
```

## 🧪 ¿Cómo probar cada componente de forma independiente?

Si algo falla, la mejor forma de depurar es aislar el problema y probar los nodos por separado:

### 1. Probar el Agente de Voz (Cerebro + Oído + Boca)
No necesitas encender los motores ni la cámara para probar si el robot platica.
1. Lanza el agente:
   ```bash
   ros2 launch llm_planning agente_SD.launch.py
   ```
2. En otra terminal, simula que la máquina de estados detectó a alguien encendiendo la interacción:
   ```bash
   ros2 topic pub -1 /interaction/enable std_msgs/msg/Bool "{data: true}"
   ```
3. Habla al micrófono. El robot debería procesar tu voz y responderte.
4. Para apagarlo manda un `false`:
   ```bash
   ros2 topic pub -1 /interaction/enable std_msgs/msg/Bool "{data: false}"
   ```

### 2. Probar solo la Boca (PiperTTS)
1. Corre únicamente el nodo de TTS:
   ```bash
   ros2 run text2speech pipertts
   ```
2. Manda un texto al tópico simulando a la IA:
   ```bash
   ros2 topic pub -1 /tts_query std_msgs/msg/String "{data: 'Hola, esta es una prueba de la voz de Eddi.'}"
   ```
   *Si no se escucha, revisa el volumen de tu sistema con `alsamixer`.*

### 3. Probar solo el Oído (Whisper)
1. Ejecuta el nodo:
   ```bash
   ros2 run speech2text faster_whisper_asr
   ```
2. Observa la terminal. Si dice "Waiting for audio...", habla fuerte.
3. En otra terminal, revisa qué texto detectó:
   ```bash
   ros2 topic echo /sp_rec/recognized
   ```

### 4. Probar la Visión (Face Detector)
1. Levanta la cámara de tu robot (ej. RealSense o USB cam).
2. Lanza el detector de caras:
   ```bash
   ros2 run face_detector face_detector
   ```
3. Abre `rqt_image_view` y selecciona el tópico `/face_detection/image` para ver si está dibujando la caja verde sobre tu rostro.
4. Verifica las coordenadas (y la distancia en Z) publicadas en el tópico:
   ```bash
   ros2 topic echo /vision/face
   ```

### 5. Probar Independientemente: Escaneo del Cuello (`deepface_follower_node`)
Este nodo corre en el paquete `person_fallower` y controla el movimiento de la cabeza (pan/tilt). Si no detecta rostros, hará un escaneo; si los detecta, centrará la vista en ellos.
1. Ejecuta el nodo:
   ```bash
   ros2 run person_fallower deepface_follower_node
   ```
2. En otra terminal, verifica que está publicando los movimientos del cuello:
   ```bash
   ros2 topic echo /hardware/head/goal_pose
   ```
3. Inyecta un rostro simulado para comprobar cómo la cabeza lo sigue:
   ```bash
   ros2 topic pub -1 /vision/face pumas_vision_msgs/msg/VisionObject "{x: 100, y: 100}"
   ```

### 6. Probar Independientemente: Seguimiento de la Base (`person_fallower`)
Este nodo calcula las velocidades de las ruedas para acercar el robot a la persona. Requiere que la señal de habilitación (`enable`) esté activa.
1. Ejecuta el nodo:
   ```bash
   ros2 run person_fallower person_fallower
   ```
2. Habilita el movimiento simulando que la máquina de estados lo requiere:
   ```bash
   ros2 topic pub -1 /person_follower/enable std_msgs/msg/Bool "{data: true}"
   ```
3. Publica una coordenada de rostro simulada (usando `pose.position.x` como distancia, ej. 1.5 metros):
   ```bash
   ros2 topic pub -1 /vision/face pumas_vision_msgs/msg/VisionObject "{x: 320, y: 240, pose: {position: {x: 1.5}}}"
   ```
4. Revisa si el nodo está publicando los comandos de velocidad hacia los motores:
   ```bash
   ros2 topic echo /cmd_vel
   ```