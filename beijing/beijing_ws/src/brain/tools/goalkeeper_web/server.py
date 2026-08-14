#!/usr/bin/env python3
"""Local web control panel for the Booster goalkeeper."""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
import pathlib
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

try:
    if os.environ.get("GOALKEEPER_WEB_OFFLINE_TEST") == "1":
        raise ImportError("ROS disabled for the offline HTTP smoke test")
    import rclpy
    from rcl_interfaces.msg import ParameterType
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
    from std_msgs.msg import String
except ImportError:  # Allows the HTTP smoke test to run without ROS installed.
    rclpy = None
    ParameterType = None
    Parameter = None
    AsyncParameterClient = None
    DurabilityPolicy = HistoryPolicy = QoSProfile = ReliabilityPolicy = None
    String = None

    class Node:
        pass


def number(label: str, group: str, default: float, minimum: float,
           maximum: float, step: float, description: str, unit: str = "") -> dict:
    return dict(label=label, group=group, type="number", default=default,
                minimum=minimum, maximum=maximum, step=step,
                description=description, unit=unit)


def integer(label: str, group: str, default: int, minimum: int,
            maximum: int, step: int, description: str, unit: str = "") -> dict:
    return dict(label=label, group=group, type="integer", default=default,
                minimum=minimum, maximum=maximum, step=step,
                description=description, unit=unit)


def boolean(label: str, group: str, default: bool, description: str) -> dict:
    return dict(label=label, group=group, type="boolean", default=default,
                description=description)


def choice(label: str, group: str, default: str, options: list[str],
           description: str) -> dict:
    return dict(label=label, group=group, type="choice", default=default,
                options=options, description=description)


SCHEMA: dict[str, dict[str, Any]] = {
    "goalkeeper.mode": choice("Modo del portero", "Bloqueo reactivo", "attack", ["attack", "guard"], "Attack puede reclamar y despejar; guard permanece en cobertura."),
    "goalkeeper.ready.dist_to_goalline": number("Posición READY", "Bloqueo reactivo", .8, .4, 1.5, .05, "Distancia desde la línea de gol para la posición inicial.", "m"),
    "goalkeeper.ready.dist_tolerance": number("Tolerancia READY", "Bloqueo reactivo", .5, .02, 1., .02, "Error de posición permitido durante READY.", "m"),
    "goalkeeper.ready.theta_tolerance": number("Tolerancia angular READY", "Bloqueo reactivo", .1, .02, 1., .02, "Error angular permitido durante READY.", "rad"),
    "goalkeeper.ready.long_range_threshold": number("Umbral lejano READY", "Bloqueo reactivo", 1., .1, 4., .1, "Distancia que cambia la estrategia de aproximación.", "m"),
    "goalkeeper.ready.turn_threshold": number("Umbral de giro READY", "Bloqueo reactivo", .4, .02, 1.5, .02, "Error angular a partir del que prioriza el giro.", "rad"),
    "goalkeeper.ready.vx_limit": number("Velocidad en READY", "Bloqueo reactivo", 1.2, 0., 2., .05, "Límite longitudinal al ir a la posición inicial.", "m/s"),
    "goalkeeper.ready.vy_limit": number("Velocidad lateral READY", "Bloqueo reactivo", .5, 0., 1.7, .05, "Límite lateral al ir a la posición inicial.", "m/s"),
    "goalkeeper.ready.vtheta_limit": number("Giro máximo READY", "Bloqueo reactivo", 1.5, 0., 1.5, .05, "Límite angular durante READY.", "rad/s"),
    "goalkeeper.ready.avoid_obstacles": boolean("Evitar obstáculos en READY", "Bloqueo reactivo", True, "Usa el planificador local al ir a la posición inicial."),
    "goalkeeper.blocking.dist_tolerance": number("Tolerancia de posición", "Bloqueo reactivo", .15, .02, 1., .01, "Distancia a partir de la cual corrige la posición defensiva.", "m"),
    "goalkeeper.blocking.theta_tolerance": number("Tolerancia angular", "Bloqueo reactivo", .30, .02, 1.2, .01, "Error angular permitido al mirar el balón.", "rad"),
    "goalkeeper.blocking.vx_limit": number("Velocidad longitudinal", "Bloqueo reactivo", .70, 0., 2., .05, "Límite hacia delante y atrás.", "m/s"),
    "goalkeeper.blocking.vy_limit": number("Velocidad lateral", "Bloqueo reactivo", .90, 0., 1.7, .05, "Límite para desplazarse hacia los postes.", "m/s"),
    "goalkeeper.blocking.vtheta_limit": number("Giro en cobertura", "Bloqueo reactivo", 1., 0., 1.5, .05, "Límite angular mientras cubre la portería.", "rad/s"),
    "goalkeeper.blocking.position_gain": number("Ganancia de cobertura", "Bloqueo reactivo", 1., 0., 5., .1, "Convierte error de posición en velocidad."),
    "goalkeeper.blocking.orientation_gain": number("Ganancia de orientación", "Bloqueo reactivo", 2., 0., 5., .1, "Convierte error al mirar el balón en giro."),
    "goalkeeper.blocking.dist_to_goalline": number("Distancia a la línea", "Bloqueo reactivo", .80, .4, 1.5, .05, "Profundidad de la línea defensiva desde la portería.", "m"),

    "goalkeeper.chase.threshold": number("Umbral de persecución", "Persecución", 1., .2, 3., .05, "Por encima de esta distancia persigue; por debajo ajusta o despeja.", "m"),
    "goalkeeper.chase.vx_limit": number("Velocidad longitudinal", "Persecución", 1.5, 0., 2., .05, "Límite hacia el balón.", "m/s"),
    "goalkeeper.chase.vy_limit": number("Velocidad lateral", "Persecución", .3, 0., 1.7, .05, "Límite lateral durante la persecución.", "m/s"),
    "goalkeeper.chase.vtheta_limit": number("Velocidad de giro", "Persecución", 1., 0., 1.5, .05, "Límite de giro durante la persecución.", "rad/s"),
    "goalkeeper.chase.target_distance": number("Distancia detrás del balón", "Persecución", .4, .1, 1.2, .05, "Separación del punto de aproximación.", "m"),
    "goalkeeper.chase.safe_distance": number("Distancia al rodear", "Persecución", .6, .2, 1.5, .05, "Radio usado al rodear el balón.", "m"),

    "goalkeeper.adjust.turn_threshold": number("Umbral de giro", "Ajuste y posesión", .2, .02, 1.5, .02, "Ángulo para priorizar el giro.", "rad"),
    "goalkeeper.adjust.range": number("Distancia de ajuste", "Ajuste y posesión", .3, .1, 1., .02, "Separación que intenta mantener antes de despejar.", "m"),
    "goalkeeper.adjust.vx_limit": number("Velocidad longitudinal", "Ajuste y posesión", .3, 0., 1.5, .05, "Límite longitudinal al alinearse.", "m/s"),
    "goalkeeper.adjust.vy_limit": number("Velocidad lateral", "Ajuste y posesión", .2, 0., 1.5, .05, "Límite lateral al rodear el balón.", "m/s"),
    "goalkeeper.adjust.vtheta_limit": number("Velocidad de giro", "Ajuste y posesión", 1., 0., 1.5, .05, "Límite angular al alinearse.", "rad/s"),
    "goalkeeper.claim.max_ball_range": number("Distancia para reclamar", "Ajuste y posesión", 1.5, .2, 4., .1, "Distancia máxima para abandonar la cobertura y tomar el balón.", "m"),
    "goalkeeper.claim.max_cost": number("Coste máximo", "Ajuste y posesión", 5., 0., 20., .25, "Coste máximo para ganar la posesión del equipo."),
    "goalkeeper.claim.extra_depth": number("Profundidad adicional", "Ajuste y posesión", 1., 0., 3., .1, "Extensión de la zona peligrosa más allá del área.", "m"),
    "goalkeeper.claim.lateral_margin": number("Margen lateral", "Ajuste y posesión", .5, 0., 2., .1, "Extensión lateral de la zona reclamable.", "m"),
    "goalkeeper.claim.require_team_lead": boolean("Exigir liderazgo del equipo", "Ajuste y posesión", True, "Sólo abandona cobertura si el portero ganó la elección distribuida del balón."),

    "goalkeeper.camera.track_tolerance_ratio": number("Tolerancia de seguimiento", "Cámara", .22, .02, .49, .01, "Fracción de la imagen antes de corregir la cabeza."),
    "goalkeeper.camera.center_tolerance_factor": number("Histéresis central", "Cámara", .8, .1, 1., .05, "Relación entre la zona de entrada y salida."),
    "goalkeeper.camera.filter_time_constant_sec": number("Filtro", "Cámara", .08, .005, .5, .005, "Suavizado de la corrección; menor reacciona más rápido.", "s"),
    "goalkeeper.camera.command_interval_sec": number("Intervalo de comando", "Cámara", .03, .01, .2, .005, "Periodo mínimo entre comandos de cabeza.", "s"),
    "goalkeeper.camera.max_pitch_rate": number("Velocidad vertical", "Cámara", .85, .05, 2., .05, "Límite vertical de la cabeza.", "rad/s"),
    "goalkeeper.camera.max_yaw_rate": number("Velocidad horizontal", "Cámara", 1.25, .05, 2., .05, "Límite horizontal de la cabeza.", "rad/s"),
    "goalkeeper.camera.min_command_change": number("Cambio mínimo", "Cámara", .004, .0005, .05, .0005, "Ignora correcciones menores a este valor.", "rad"),
    "goalkeeper.camera.search_cycle_msec": integer("Ciclo de búsqueda", "Cámara", 3000, 800, 10000, 100, "Duración de un barrido completo.", "ms"),
    "goalkeeper.camera.search_max_pitch_rate": number("Búsqueda vertical", "Cámara", .8, .05, 2., .05, "Velocidad vertical durante búsqueda.", "rad/s"),
    "goalkeeper.camera.search_max_yaw_rate": number("Búsqueda horizontal", "Cámara", 1.4, .05, 2., .05, "Velocidad horizontal durante búsqueda.", "rad/s"),

    "goalkeeper.kick.type": choice("Tipo de despeje", "Patada", "default", ["default", "visual"], "Selecciona Kick convencional o VisualKick de la SDK."),
    "goalkeeper.kick.alignment_tolerance": number("Alineación para patear", "Patada", 1.5708, .02, 3.1416, .02, "Diferencia máxima entre la dirección robot-balón y el despeje.", "rad"),
    "goalkeeper.kick.default.speed_limit": number("Velocidad Kick", "Patada", 1.2, .1, 2., .05, "Velocidad máxima del crab-walk convencional.", "m/s"),
    "goalkeeper.kick.default.min_msec": number("Duración mínima Kick", "Patada", 600., 100., 5000., 50., "Tiempo base del despeje convencional.", "ms"),
    "goalkeeper.kick.default.enable_stabilize": boolean("Estabilizar Kick", "Patada", False, "Retrocede y estabiliza antes del despeje convencional."),
    "goalkeeper.kick.default.stabilize_msec": number("Tiempo de estabilización", "Patada", 1000., 0., 4000., 50., "Espera previa cuando la estabilización está activa.", "ms"),
    "goalkeeper.kick.default.exit_range": number("Rango de salida Kick", "Patada", 1., .2, 3., .05, "Finaliza el despeje si el balón queda más lejos.", "m"),
    "goalkeeper.kick.default.abort_when_ball_moved": boolean("Terminar al mover balón", "Patada", True, "Finaliza cuando confirma que el balón salió del punto inicial."),
    "goalkeeper.kick.default.ball_move_threshold": number("Movimiento confirmado", "Patada", .3, .15, 1., .05, "Desplazamiento que confirma el despeje.", "m"),
    "goalkeeper.kick.visual.min_msec": number("VisualKick mínimo", "Patada", 1500., 0., 10000., 100., "Tiempo mínimo que mantiene VisualKick.", "ms"),
    "goalkeeper.kick.visual.max_msec": number("VisualKick máximo", "Patada", 5000., 500., 20000., 100., "Timeout de VisualKick.", "ms"),
    "goalkeeper.kick.visual.range": number("Rango de salida VisualKick", "Patada", 2., .2, 6., .1, "Termina si el balón detectado supera este rango.", "m"),
    "goalkeeper.kick.visual.pre_delay_msec": number("Espera previa VisualKick", "Patada", 1000., 0., 3000., 50., "Detención antes de entrar a VisualKick.", "ms"),
    "goalkeeper.kick.visual.post_delay_msec": number("Espera de salida VisualKick", "Patada", 1000., 0., 3000., 50., "Detención al salir de VisualKick.", "ms"),
    "RLVisionKick.visual_kick_version": choice("Versión VisualKick", "Patada", "kV2", ["kV1", "kV2"], "Versión de la API VisualKick de Booster."),
    "obstacle_avoidance.avoid_during_kick": boolean("Evitar obstáculos en Kick", "Patada", False, "Aborta el despeje convencional si detecta un obstáculo frontal."),
    "obstacle_avoidance.kick_ao_safe_dist": number("Seguridad de Kick", "Patada", 1.5, .2, 4., .1, "Distancia frontal de seguridad durante Kick.", "m"),
    "obstacle_avoidance.enable_fallen_robot_visual_kick_exit": boolean("Salir por robot caído", "Patada", True, "Impide o termina VisualKick ante un robot caído."),
    "obstacle_avoidance.fallen_robot_distance": number("Distancia robot caído", "Patada", 3., .2, 6., .1, "Alcance de protección de VisualKick.", "m"),
    "obstacle_avoidance.fallen_robot_angle": number("Ángulo robot caído", "Patada", .785, .1, 1.57, .05, "Semisector frontal de protección.", "rad"),
    "obstacle_avoidance.avoid_during_chase": boolean("Evitar durante persecución", "Persecución", True, "Activa el planificador local al perseguir."),
    "obstacle_avoidance.chase_ao_safe_dist": number("Seguridad de persecución", "Persecución", 1.5, .2, 4., .1, "Distancia de seguridad durante Chase.", "m"),

    "goalkeeper.prediction.enabled": boolean("Activar predictor", "Predicción", False, "Habilita detección de tiros y el estado block_shot."),
    "goalkeeper.prediction.require_localization": boolean("Exigir localización", "Predicción", True, "Impide mover al portero por una predicción hasta que odom_calibrated sea verdadero."),
    "goalkeeper.prediction.history_msec": number("Ventana histórica", "Predicción", 900., 200., 3000., 50., "Antigüedad máxima de observaciones.", "ms"),
    "goalkeeper.prediction.max_samples": integer("Muestras máximas", "Predicción", 12, 3, 50, 1, "Cantidad máxima de observaciones retenidas."),
    "goalkeeper.prediction.min_samples": integer("Muestras mínimas", "Predicción", 5, 3, 20, 1, "Cantidad mínima para aceptar una velocidad."),
    "goalkeeper.prediction.min_span_msec": number("Intervalo mínimo", "Predicción", 250., 50., 1500., 25., "Tiempo mínimo cubierto por las muestras.", "ms"),
    "goalkeeper.prediction.min_speed": number("Velocidad mínima", "Predicción", .45, 0., 5., .05, "Ignora balones demasiado lentos.", "m/s"),
    "goalkeeper.prediction.min_toward_goal_speed": number("Componente hacia portería", "Predicción", .35, 0., 5., .05, "Velocidad x negativa mínima para considerarlo tiro.", "m/s"),
    "goalkeeper.prediction.min_r_squared": number("Calidad lineal R²", "Predicción", .90, 0., 1., .01, "Calidad mínima del ajuste temporal."),
    "goalkeeper.prediction.max_residual": number("Residual máximo", "Predicción", .20, .01, 1., .01, "Error RMS máximo de la trayectoria.", "m"),
    "goalkeeper.prediction.max_sample_jump": number("Salto máximo", "Predicción", .80, .05, 3., .05, "Reinicia el historial ante saltos de percepción.", "m"),
    "goalkeeper.prediction.min_ball_confidence": number("Confianza mínima", "Predicción", 40., 0., 100., 1., "Descarta detecciones débiles antes del ajuste.", "%"),
    "goalkeeper.prediction.recency_weight": number("Peso de muestras recientes", "Predicción", 2., 1., 10., .25, "Da más influencia a las detecciones nuevas para reducir la latencia."),
    "goalkeeper.prediction.deceleration": number("Desaceleración", "Predicción", .40, 0., 3., .05, "Fricción supuesta sobre el balón.", "m/s²"),
    "goalkeeper.prediction.step_interval_msec": number("Paso de trayectoria", "Predicción", 100., 20., 500., 10., "Resolución de puntos futuros.", "ms"),
    "goalkeeper.prediction.step_count": integer("Cantidad de pasos", "Predicción", 30, 1, 100, 1, "Máximo de puntos futuros."),
    "goalkeeper.prediction.goal_margin": number("Margen de postes", "Predicción", .15, 0., 1., .02, "Margen exterior para considerar amenaza.", "m"),
    "goalkeeper.prediction.min_time_to_block": number("Tiempo mínimo", "Predicción", .08, 0., 1., .01, "Descarta cruces ya ocurridos o demasiado inmediatos.", "s"),
    "goalkeeper.prediction.max_time_to_block": number("Horizonte máximo", "Predicción", 2.5, .1, 8., .1, "Solo bloquea tiros dentro de este horizonte.", "s"),
    "goalkeeper.prediction.activation_hold_msec": number("Retención de amenaza", "Predicción", 250., 0., 1500., 25., "Evita alternar block_shot por una detección perdida.", "ms"),
    "goalkeeper.prediction.block.vx_limit": number("Bloqueo longitudinal", "Bloqueo predictivo", .7, 0., 2., .05, "Límite longitudinal durante block_shot.", "m/s"),
    "goalkeeper.prediction.block.vy_limit": number("Bloqueo lateral", "Bloqueo predictivo", 1., 0., 1.7, .05, "Límite lateral durante block_shot.", "m/s"),
    "goalkeeper.prediction.block.vtheta_limit": number("Giro en bloqueo", "Bloqueo predictivo", 1., 0., 1.5, .05, "Límite angular durante block_shot.", "rad/s"),
    "goalkeeper.prediction.block.position_gain": number("Ganancia de posición", "Bloqueo predictivo", 1.5, .1, 5., .1, "Convierte el error de posición en velocidad."),
    "goalkeeper.prediction.block.reaction_margin_sec": number("Margen de reacción", "Bloqueo predictivo", .12, 0., 1., .01, "Tiempo reservado a latencia y aceleración.", "s"),
    "goalkeeper.prediction.block.target_tolerance": number("Tolerancia del punto", "Bloqueo predictivo", .10, .02, .5, .01, "Distancia para considerar alcanzado el punto.", "m"),
    "goalkeeper.prediction.block.apply_min_velocity": boolean("Compensar zona muerta", "Bloqueo predictivo", True, "Aplica la velocidad mínima configurada para iniciar movimiento."),
}


GROUPS = ["Bloqueo reactivo", "Persecución", "Ajuste y posesión", "Cámara", "Patada", "Predicción", "Bloqueo predictivo"]

# This file is deliberately separate from config_local.yaml: the GUI may
# overwrite the latter, but never mutates the original recovery profile.
FACTORY_DEFAULTS_PATH = pathlib.Path(__file__).with_name(
    "factory_defaults.json")
FACTORY_DEFAULTS: dict[str, Any] = json.loads(
    FACTORY_DEFAULTS_PATH.read_text(encoding="utf-8"))
if set(FACTORY_DEFAULTS) != set(SCHEMA):
    missing = sorted(set(SCHEMA) - set(FACTORY_DEFAULTS))
    extra = sorted(set(FACTORY_DEFAULTS) - set(SCHEMA))
    raise RuntimeError(
        f"Perfil original desactualizado; faltan={missing}, sobran={extra}")


class ParameterTransport:
    """Narrow interface shared by the ROS bridge and the offline test bridge."""

    def snapshot(self) -> dict[str, Any]:
        raise NotImplementedError

    def read_values(self) -> dict[str, Any]:
        raise NotImplementedError

    def apply_values(self, values: dict[str, Any]) -> list[str]:
        raise NotImplementedError


class GoalkeeperBridge(Node):
    def __init__(self, target_node: str, log_dir: pathlib.Path):
        super().__init__("goalkeeper_web_bridge")
        self.target_node = target_node
        self.client = AsyncParameterClient(self, target_node)
        self.parameter_lock = threading.RLock()
        self.status_lock = threading.Lock()
        self.status: dict[str, Any] = {"connected": False, "decision": "sin datos"}
        self.telemetry = deque(maxlen=2000)
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = log_dir / f"goalkeeper_telemetry_{stamp}.jsonl"
        self.log_file = self.log_path.open("a", encoding="utf-8", buffering=1)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/brain/goalkeeper/status", self._status, qos)

    def _status(self, msg: String) -> None:
        try:
            decoded = json.loads(msg.data)
        except json.JSONDecodeError:
            decoded = {"raw": msg.data}
        decoded["connected"] = True
        decoded["received_at"] = time.time()
        decoded["received_at_iso"] = datetime.now(timezone.utc).isoformat()
        with self.status_lock:
            self.status = decoded
            self.telemetry.append(dict(decoded))
            self.log_file.write(json.dumps(decoded, ensure_ascii=False) + "\n")

    def snapshot(self) -> dict[str, Any]:
        with self.status_lock:
            result = dict(self.status)
        if result.get("received_at", 0) < time.time() - 2.0:
            result["connected"] = False
        return result

    def telemetry_snapshot(self, limit: int = 200) -> list[dict[str, Any]]:
        with self.status_lock:
            return list(self.telemetry)[-max(1, min(limit, 2000)):]

    def close(self) -> None:
        self.log_file.close()
        self.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _wait(future, timeout: float = 4.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        if not future.done():
            raise TimeoutError("El nodo brain no respondió a tiempo")
        if future.exception():
            raise future.exception()
        return future.result()

    def read_values(self) -> dict[str, Any]:
        with self.parameter_lock:
            if not self.client.wait_for_services(timeout_sec=1.0):
                raise ConnectionError("No se encontró /brain_node")
            response = self._wait(self.client.get_parameters(list(SCHEMA)))
            values = {}
            for name, value in zip(SCHEMA, response.values):
                if value.type == ParameterType.PARAMETER_BOOL:
                    values[name] = value.bool_value
                elif value.type == ParameterType.PARAMETER_INTEGER:
                    values[name] = value.integer_value
                elif value.type == ParameterType.PARAMETER_DOUBLE:
                    values[name] = value.double_value
                elif value.type == ParameterType.PARAMETER_STRING:
                    values[name] = value.string_value
                else:
                    values[name] = SCHEMA[name]["default"]
            return values

    def apply_values(self, values: dict[str, Any]) -> list[str]:
        with self.parameter_lock:
            normalized: dict[str, Any] = {}
            for name, raw in values.items():
                if name not in SCHEMA:
                    raise ValueError(f"Parámetro no permitido: {name}")
                spec = SCHEMA[name]
                if spec["type"] == "boolean":
                    if not isinstance(raw, bool):
                        raise ValueError(f"{name} debe ser booleano")
                    value = raw
                elif spec["type"] == "integer":
                    if isinstance(raw, bool):
                        raise ValueError(f"{name} debe ser entero")
                    value = int(raw)
                    if value < spec["minimum"] or value > spec["maximum"]:
                        raise ValueError(f"{name} está fuera de rango")
                elif spec["type"] == "number":
                    if isinstance(raw, bool):
                        raise ValueError(f"{name} debe ser numérico")
                    value = float(raw)
                    if not math.isfinite(value) or value < spec["minimum"] or value > spec["maximum"]:
                        raise ValueError(f"{name} está fuera de rango")
                else:
                    value = str(raw)
                    if value not in spec["options"]:
                        raise ValueError(f"Valor no permitido para {name}")
                normalized[name] = value

            candidate = self.read_values()
            candidate.update(normalized)
            validate_relationships(candidate)
            parameters = [
                Parameter(name=name, value=value)
                for name, value in normalized.items()
            ]
            response = self._wait(
                self.client.set_parameters_atomically(parameters))
            if not response.result.successful:
                raise RuntimeError(response.result.reason or
                                   "brain rechazó los parámetros")
            return list(normalized)


def validate_relationships(values: dict[str, Any]) -> None:
    pairs = [
        ("goalkeeper.kick.visual.min_msec",
         "goalkeeper.kick.visual.max_msec",
         "VisualKick mínimo no puede superar el máximo"),
        ("goalkeeper.prediction.min_samples",
         "goalkeeper.prediction.max_samples",
         "Las muestras mínimas no pueden superar las máximas"),
        ("goalkeeper.prediction.min_span_msec",
         "goalkeeper.prediction.history_msec",
         "El intervalo mínimo no puede superar la ventana histórica"),
        ("goalkeeper.prediction.min_time_to_block",
         "goalkeeper.prediction.max_time_to_block",
         "El tiempo mínimo de bloqueo no puede superar el máximo"),
    ]
    for lower, upper, message in pairs:
        if values[lower] > values[upper]:
            raise ValueError(message)


def nest_parameters(values: dict[str, Any]) -> dict[str, Any]:
    root: dict[str, Any] = {}
    for dotted, value in values.items():
        cursor = root
        parts = dotted.split(".")
        for part in parts[:-1]:
            cursor = cursor.setdefault(part, {})
        cursor[parts[-1]] = value
    return {"brain_node": {"ros__parameters": root}}


def atomic_yaml_write(path: pathlib.Path, values: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    def scalar(value: Any) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, str):
            return json.dumps(value, ensure_ascii=False)
        return str(value)

    def lines(mapping: dict[str, Any], depth: int = 0) -> list[str]:
        output: list[str] = []
        prefix = "  " * depth
        for key, value in mapping.items():
            if isinstance(value, dict):
                output.append(f"{prefix}{key}:")
                output.extend(lines(value, depth + 1))
            else:
                output.append(f"{prefix}{key}: {scalar(value)}")
        return output

    content = "\n".join(lines(nest_parameters(values))) + "\n"
    temp.write_text(content, encoding="utf-8")
    os.replace(temp, path)


class ApiHandler(SimpleHTTPRequestHandler):
    bridge: ParameterTransport
    static_dir: pathlib.Path
    config_paths: list[pathlib.Path]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(self.static_dir), **kwargs)

    def _json(self, status: int, payload: Any) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        try:
            if self.path == "/api/schema":
                return self._json(200, {"groups": GROUPS, "parameters": SCHEMA})
            if self.path == "/api/factory-defaults":
                return self._json(200, {
                    "profile": "demo-original-safe-v1",
                    "values": FACTORY_DEFAULTS,
                    "notes": (
                        "Comportamiento original del demo: modo attack, patada "
                        "default y predictor desactivado. Los controles nuevos "
                        "usan sus valores iniciales seguros.")
                })
            if self.path == "/api/config":
                return self._json(200, {"values": self.bridge.read_values()})
            if self.path == "/api/status":
                return self._json(200, self.bridge.snapshot())
            if self.path.startswith("/api/telemetry"):
                try:
                    limit = int(self.path.partition("limit=")[2] or "200")
                except ValueError:
                    limit = 200
                return self._json(200, {
                    "events": self.bridge.telemetry_snapshot(limit),
                    "log_file": str(self.bridge.log_path),
                })
            if self.path == "/api/log/download":
                path = self.bridge.log_path
                data = path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "application/x-ndjson")
                self.send_header(
                    "Content-Disposition", f'attachment; filename="{path.name}"')
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
                return
        except Exception as exc:
            return self._json(503, {"error": str(exc)})
        return super().do_GET()

    def do_POST(self):
        if self.path != "/api/apply":
            return self._json(404, {"error": "Ruta desconocida"})
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 1024 * 1024:
                raise ValueError("Tamaño de solicitud no permitido")
            body = json.loads(self.rfile.read(length) or b"{}")
            values = body.get("values", {})
            if not isinstance(values, dict) or not values:
                raise ValueError("No se recibieron parámetros")
            candidate = self.bridge.read_values()
            candidate.update(values)
            validate_relationships(candidate)
            applied = self.bridge.apply_values(values)
            saved = []
            if body.get("persist", False):
                complete = self.bridge.read_values()
                for path in self.config_paths:
                    atomic_yaml_write(path, complete)
                    saved.append(str(path))
            self._json(200, {"ok": True, "applied": applied, "saved": saved})
        except (ValueError, RuntimeError) as exc:
            self._json(400, {"error": str(exc)})
        except Exception as exc:
            self._json(503, {"error": str(exc)})

    def log_message(self, fmt: str, *args) -> None:
        print("[goalkeeper-web]", fmt % args)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--node", default="/brain_node")
    parser.add_argument("--config", action="append", default=[])
    parser.add_argument(
        "--log-dir", default="goalkeeper_logs",
        help="Directorio para telemetria JSONL persistente")
    args = parser.parse_args()

    if rclpy is None:
        raise RuntimeError(
            "rclpy no está disponible; ejecute después de source install/setup.bash")
    rclpy.init()
    bridge = GoalkeeperBridge(
        args.node, pathlib.Path(args.log_dir).expanduser().resolve())
    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()

    ApiHandler.bridge = bridge
    ApiHandler.static_dir = pathlib.Path(__file__).with_name("static")
    ApiHandler.config_paths = list(dict.fromkeys(
        pathlib.Path(item).expanduser().resolve() for item in args.config))
    server = ThreadingHTTPServer((args.host, args.port), ApiHandler)
    print(f"Goalkeeper web panel: http://{args.host}:{args.port}")
    print(f"Goalkeeper telemetry log: {bridge.log_path}")
    try:
        server.serve_forever()
    finally:
        server.server_close()
        bridge.close()


if __name__ == "__main__":
    main()
