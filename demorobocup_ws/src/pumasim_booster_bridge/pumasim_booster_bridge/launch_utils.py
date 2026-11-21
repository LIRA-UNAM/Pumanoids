"""Launch parameter loading and remapping helpers for Pumanoids RoboCup demo."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml


def flatten_ros_parameters(
    mapping: Mapping[str, Any], prefix: str = ""
) -> dict[str, Any]:
    """Flatten nested ROS 2 YAML mappings into dotted parameter keys."""

    flattened: dict[str, Any] = {}
    for key, value in mapping.items():
        if not isinstance(key, str) or not key:
            raise ValueError(f"ROS parameter key must be a non-empty string, got {key!r}")
        dotted = f"{prefix}.{key}" if prefix else key
        if isinstance(value, Mapping):
            flattened.update(flatten_ros_parameters(value, dotted))
        else:
            flattened[dotted] = value
    return flattened


def load_node_parameters_from_yaml(
    yaml_path: str | Path,
    *,
    expected_node_key: str = "brain_node",
) -> dict[str, Any]:
    """Load ``ros__parameters`` from a node-keyed YAML file into a flat parameter dict.

    Passing flattened dicts to ``launch_ros.actions.Node(..., parameters=[...])``
    causes ``launch_ros`` to serialize them under the launched node's actual name
    and namespace (e.g. ``brain_blue_1``), preserving base -> overlay -> override
    precedence even when the YAML files use ``brain_node:``.
    """

    path = Path(yaml_path)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise TypeError(f"Parameter YAML {path} must define a mapping at top level")

    section = raw.get(expected_node_key)
    if section is None and "/**" in raw:
        section = raw["/**"]
    if section is None:
        raise ValueError(
            f"Parameter YAML {path} is missing '{expected_node_key}.ros__parameters'"
        )
    if not isinstance(section, Mapping) or not isinstance(
        section.get("ros__parameters"), Mapping
    ):
        raise TypeError(
            f"Parameter YAML {path} must define '{expected_node_key}.ros__parameters' as a mapping"
        )
    return flatten_ros_parameters(section["ros__parameters"])
