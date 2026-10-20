import json
import time

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _num(name, label, unit, lo, hi, step):
    return {"name": name, "label": label, "unit": unit, "type": "double",
            "min": lo, "max": hi, "step": step}


# Scalar hyperparameters of t1_ppowalk_policy_node (t1_ppowalk/config/t1_ppowalk_policy.yaml).
POLICY_SPECS = [
    _num("max_start_tilt_deg", "Max tilt to allow Start", "deg", 1.0, 90.0, 1.0),
    _num("start_ramp_s", "Start ramp (blend into policy)", "s", 0.0, 10.0, 0.1),
    _num("home_max_vel", "Home: peak joint speed", "rad/s", 0.05, 2.0, 0.05),
    _num("home_min_duration_s", "Home: minimum duration", "s", 0.0, 20.0, 0.5),
    _num("action_clip", "Action clip (0 = off)", "", 0.0, 100.0, 0.1),
    _num("policy_rate_hz", "Policy rate", "Hz", 1.0, 1000.0, 1.0),
    _num("command_rate_hz", "LowCmd rate", "Hz", 1.0, 2000.0, 1.0),
    _num("data_timeout_s", "LowState timeout", "s", 0.01, 2.0, 0.01),
    _num("lin_vel_x_min", "vx min", "m/s", -3.0, 0.0, 0.05),
    _num("lin_vel_x_max", "vx max", "m/s", 0.0, 3.0, 0.05),
    _num("lin_vel_y_min", "vy min", "m/s", -3.0, 0.0, 0.05),
    _num("lin_vel_y_max", "vy max", "m/s", 0.0, 3.0, 0.05),
    _num("ang_vel_z_min", "wz min", "rad/s", -3.0, 0.0, 0.05),
    _num("ang_vel_z_max", "wz max", "rad/s", 0.0, 3.0, 0.05),
    {"name": "detect_offline", "label": "Damp + pin obs of motors with error flags", "unit": "",
     "type": "bool"},
    _num("cmd_vel_timeout_s", "cmd_vel timeout (falls back to 0)", "s", 0.05, 5.0, 0.05),
]

# Velocity-command calibration (shown in the command panel).
OFFSET_SPECS = [
    _num("cmd_vx_offset", "vx offset", "m/s", -0.5, 0.5, 0.01),
    _num("cmd_vy_offset", "vy offset", "m/s", -0.5, 0.5, 0.01),
    _num("cmd_wz_offset", "wz offset", "rad/s", -0.5, 0.5, 0.01),
    {"name": "offsets_when_standing", "label": "Apply offsets with a zero command", "unit": "",
     "type": "bool"},
]

# Booster remote controller (shown in the command panel).
REMOTE_SPECS = [
    {"name": "remote_start", "label": "START (+) starts the policy", "unit": "",
     "type": "bool"},
    {"name": "remote_home", "label": "BACK (-) moves home", "unit": "", "type": "bool"},
    _num("remote_deadzone", "Stick deadzone", "", 0.0, 0.9, 0.01),
    _num("remote_timeout_s", "Remote data timeout (falls back to 0)", "s", 0.05, 5.0, 0.05),
]

# Velocity command source -> (use_remote, use_cmd_vel); set atomically.
SOURCE_PARAMS = ["use_remote", "use_cmd_vel"]
COMMAND_SOURCES = {"remote": (True, False), "gui": (False, False), "cmd_vel": (False, True)}

# Motor output parameters (the "bridge" panel). The T1 policy node publishes
# LowCmd itself, so these live on the policy node too.
BRIDGE_SPECS = [
    # Shown in the top group's Motor output tile: keep the labels short.
    _num("damping_kd", "Damping Kd", "N·m·s/rad", 0.0, 20.0, 0.1),
    _num("max_target_delta", "Max |target − q|", "rad, 0 = off", 0.0, 6.0, 0.01),
]

# Per-joint arrays of the joint table. "index_by" names the parameter whose
# joint names index the array (action_scale does not cover the neck).
JOINT_COLUMNS = [
    {"node": "policy", "name": "default_joint_pos", "label": "Default pos", "unit": "rad",
     "step": 0.001, "index_by": "joint_names"},
    {"node": "policy", "name": "home_pose", "label": "Home pose", "unit": "rad",
     "step": 0.01, "index_by": "joint_names"},
    {"node": "policy", "name": "held_joint_targets", "label": "Hold target", "unit": "rad",
     "step": 0.01, "index_by": "joint_names"},
    {"node": "policy", "name": "action_scale", "label": "Action scale", "unit": "",
     "step": 0.001, "index_by": "action_joint_names"},
    {"node": "bridge", "name": "kp", "label": "Kp", "unit": "N·m/rad", "step": 0.1,
     "index_by": "joint_names"},
    {"node": "bridge", "name": "kd", "label": "Kd", "unit": "N·m·s/rad", "step": 0.01,
     "index_by": "joint_names"},
    {"node": "bridge", "name": "effort_limit", "label": "Effort (sim)", "unit": "N·m",
     "step": 1.0, "index_by": "joint_names", "readonly": True},
    {"node": "bridge", "name": "position_lower", "label": "Lower limit", "unit": "rad",
     "step": 0.01, "index_by": "joint_names"},
    {"node": "bridge", "name": "position_upper", "label": "Upper limit", "unit": "rad",
     "step": 0.01, "index_by": "joint_names"},
]

COMMAND_PARAMS = ["cmd_vx", "cmd_vy", "cmd_wz"]
# Joint-name lists shown as checkbox columns of the joint table.
JOINT_FLAGS = [
    {"name": "command_joints", "label": "Commanded",
     "title": "Commanded with Kp/Kd (others are damped). Only while the policy is stopped."},
    {"name": "obs_pinned_joints", "label": "Pin obs",
     "title": "Report this joint at its hold target, at rest, in the observation."},
]
POLICY_EXTRA = ["joint_names", "action_joint_names", "observation_terms", "observation_scales",
                "policy_path", "always_commanded_joints"] + COMMAND_PARAMS + SOURCE_PARAMS + [
                    f["name"] for f in JOINT_FLAGS]
BRIDGE_EXTRA = ["joint_names", "enable_lowcmd"]

TYPE_MAP = {
    "double": (Parameter.Type.DOUBLE, float),
    "bool": (Parameter.Type.BOOL, bool),
    "string": (Parameter.Type.STRING, str),
    "double_array": (Parameter.Type.DOUBLE_ARRAY, lambda v: [float(x) for x in v]),
    "string_array": (Parameter.Type.STRING_ARRAY, lambda v: [str(x) for x in v]),
}


def _param_types():
    types = {"policy": {}, "bridge": {}}
    for spec in POLICY_SPECS + OFFSET_SPECS + REMOTE_SPECS:
        types["policy"][spec["name"]] = spec["type"]
    for name in SOURCE_PARAMS:
        types["policy"][name] = "bool"
    for spec in BRIDGE_SPECS:
        types["bridge"][spec["name"]] = spec["type"]
    for col in JOINT_COLUMNS:
        if not col.get("readonly"):
            types[col["node"]][col["name"]] = "double_array"
    for name in COMMAND_PARAMS:
        types["policy"][name] = "double"
    for flag in JOINT_FLAGS:
        types["policy"][flag["name"]] = "string_array"
    types["policy"]["observation_scales"] = "double_array"
    types["policy"]["policy_path"] = "string"
    types["bridge"]["enable_lowcmd"] = "bool"
    return types


PARAM_TYPES = _param_types()
TRIGGERS = ("start", "stop", "release", "home", "enter_custom_mode", "exit_custom_mode")
SERVICE_WAIT_SEC = 1.0
SERVICE_TIMEOUT_SEC = 5.0
STATUS_STALE_SEC = 0.5


def _output_state(policy):
    """Motor output state for the "bridge" panel, from the policy status."""
    if not policy.get("enable_lowcmd"):
        return "disabled"
    if not policy.get("data_fresh"):
        return "damping (no LowState)"
    return "damping" if policy.get("mode") == "IDLE" else "tracking"


class T1PpowalkGuiBridge(Node):
    """ROS side of the T1 ppowalk GUI (served by t1_ppowalk_gui.web.run_server)."""

    def __init__(self):
        super().__init__("t1_ppowalk_gui_bridge")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8084)
        self.declare_parameter("policy_node", "/t1_ppowalk_policy_node")
        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        policy_node = self.get_parameter("policy_node").value
        # Both panels talk to the policy node (it also drives the motors).
        self._targets = {"policy": policy_node, "bridge": policy_node}
        get_client = self.create_client(GetParameters, policy_node + "/get_parameters")
        set_client = self.create_client(
            SetParametersAtomically, policy_node + "/set_parameters_atomically")
        self._get_clients = {"policy": get_client, "bridge": get_client}
        self._set_clients = {"policy": set_client, "bridge": set_client}
        self._triggers = {a: self.create_client(Trigger, "%s/%s" % (policy_node, a))
                          for a in TRIGGERS}
        self._status = (None, 0.0)
        self.create_subscription(String, policy_node + "/status", self._on_status, 10)

    def config(self):
        return {"policy_node": self._targets["policy"], "bridge_node": "motor output",
                "policy_specs": POLICY_SPECS, "offset_specs": OFFSET_SPECS,
                "remote_specs": REMOTE_SPECS,
                "bridge_specs": BRIDGE_SPECS,
                "joint_columns": JOINT_COLUMNS, "joint_flags": JOINT_FLAGS}

    def status_snapshot(self):
        status, stamp = self._status
        online = status is not None and time.monotonic() - stamp < STATUS_STALE_SEC
        policy = dict(status or {}, online=online)
        bridge = {"online": online, "state": _output_state(policy),
                  "enable_lowcmd": bool(policy.get("enable_lowcmd")),
                  "offline_joints": policy.get("offline_joints", [])}
        return {"policy": policy, "bridge": bridge}

    def _on_status(self, msg):
        try:
            self._status = (json.loads(msg.data), time.monotonic())
        except ValueError:
            self.get_logger().warning("invalid policy status")

    # -- blocking commands (worker threads, see t1_ppowalk_gui.web) -------------

    def execute_command(self, command):
        action = command.get("action")
        if action == "read_parameters":
            return self._read_all()
        if action == "set_parameters":
            return self._set_target_parameters(command.get("node"), command.get("parameters") or {})
        if action == "set_command":
            values = {name: command.get(key, 0.0)
                      for name, key in zip(COMMAND_PARAMS, ("vx", "vy", "wz"))}
            return self._set_target_parameters("policy", values)
        if action == "set_source":
            source = command.get("source")
            if source not in COMMAND_SOURCES:
                raise ValueError("unknown_source:%s" % source)
            return self._set_target_parameters(
                "policy", dict(zip(SOURCE_PARAMS, COMMAND_SOURCES[source])))
        if action in TRIGGERS:
            result = self._wait_and_call(self._triggers[action], Trigger.Request())
            return {"ok": result.success, "message": result.message}
        raise ValueError("unknown_action:%s" % action)

    def _wait_and_call(self, client, request):
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            raise RuntimeError("service_unavailable:%s" % client.srv_name)
        future = client.call_async(request)
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done() or future.result() is None:
            raise RuntimeError("service_failed:%s" % client.srv_name)
        return future.result()

    def _read_all(self):
        names = {
            "policy": [s["name"] for s in POLICY_SPECS + OFFSET_SPECS + REMOTE_SPECS] +
            POLICY_EXTRA + [
                c["name"] for c in JOINT_COLUMNS if c["node"] == "policy"],
            "bridge": [s["name"] for s in BRIDGE_SPECS] + BRIDGE_EXTRA + [
                c["name"] for c in JOINT_COLUMNS if c["node"] == "bridge"],
        }
        values, errors = {}, {}
        for key, keys in names.items():
            try:
                request = GetParameters.Request(names=keys)
                result = self._wait_and_call(self._get_clients[key], request)
                values[key] = {n: self._decode(v) for n, v in zip(keys, result.values)}
            except RuntimeError as exc:
                errors[key] = str(exc)
        return {"ok": bool(values), "parameters": values, "errors": errors}

    @staticmethod
    def _decode(value):
        t = value.type
        if t == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        if t == ParameterType.PARAMETER_INTEGER:
            return value.integer_value
        if t == ParameterType.PARAMETER_BOOL:
            return value.bool_value
        if t == ParameterType.PARAMETER_STRING:
            return value.string_value
        if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(value.double_array_value)
        if t == ParameterType.PARAMETER_STRING_ARRAY:
            return list(value.string_array_value)
        return None

    def _set_target_parameters(self, key, values):
        if key not in self._targets:
            raise ValueError("unknown_node:%s" % key)
        types = PARAM_TYPES[key]
        unknown = [n for n in values if n not in types]
        if unknown:
            raise ValueError("unknown_parameters:%s" % ",".join(unknown))
        request = SetParametersAtomically.Request()
        try:
            request.parameters = [
                Parameter(n, TYPE_MAP[types[n]][0], TYPE_MAP[types[n]][1](v)).to_parameter_msg()
                for n, v in values.items()]
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid_value:%s" % exc)
        result = self._wait_and_call(self._set_clients[key], request).result
        return {"ok": result.successful, "node": key, "reason": result.reason}
