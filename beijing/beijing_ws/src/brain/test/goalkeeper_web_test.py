#!/usr/bin/env python3
"""Integration smoke test for the goalkeeper HTTP API without ROS hardware."""

from __future__ import annotations

import argparse
import http.client
import json
import os
import pathlib
import sys
import tempfile
import threading
from http.server import ThreadingHTTPServer
from typing import Any


class FakeBridge:
    def __init__(self, schema: dict[str, dict[str, Any]]):
        self.values = {name: spec["default"] for name, spec in schema.items()}
        self.log_path = pathlib.Path("fake_goalkeeper_telemetry.jsonl")

    def snapshot(self) -> dict[str, Any]:
        return {
            "connected": True,
            "decision": "block_shot",
            "prediction_enabled": True,
            "prediction_valid": True,
            "sample_count": 7,
        }

    def read_values(self) -> dict[str, Any]:
        return dict(self.values)

    def telemetry_snapshot(self, limit: int = 200) -> list[dict[str, Any]]:
        return [{
            "decision": "block_shot",
            "prediction_reason": "threat_detected",
            "sample_count": 7,
        }][-limit:]

    def apply_values(self, values: dict[str, Any]) -> list[str]:
        self.values.update(values)
        return list(values)

    def close(self) -> None:
        pass


def request(base: tuple[str, int], path: str,
            payload: dict[str, Any] | None = None):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    connection = http.client.HTTPConnection(*base, timeout=3)
    try:
        connection.request(
            "GET" if data is None else "POST",
            path,
            body=data,
            headers={"Content-Type": "application/json"},
        )
        response = connection.getresponse()
        return response.status, json.loads(response.read().decode("utf-8"))
    finally:
        connection.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--server-dir", required=True)
    args = parser.parse_args()
    server_dir = pathlib.Path(args.server_dir).resolve()
    sys.path.insert(0, str(server_dir))
    os.environ["GOALKEEPER_WEB_OFFLINE_TEST"] = "1"
    import server as goalkeeper_server

    with tempfile.TemporaryDirectory() as temp_dir:
        config_path = pathlib.Path(temp_dir) / "config_local.yaml"
        goalkeeper_server.ApiHandler.bridge = FakeBridge(
            goalkeeper_server.SCHEMA)
        goalkeeper_server.ApiHandler.static_dir = server_dir / "static"
        goalkeeper_server.ApiHandler.config_paths = [config_path]
        httpd = ThreadingHTTPServer(
            ("127.0.0.1", 0), goalkeeper_server.ApiHandler)
        thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        thread.start()
        base = ("127.0.0.1", httpd.server_port)
        try:
            status, schema = request(base, "/api/schema")
            assert status == 200 and len(schema["parameters"]) >= 90
            status, factory = request(base, "/api/factory-defaults")
            assert status == 200
            assert len(factory["values"]) == len(schema["parameters"])
            assert factory["values"]["goalkeeper.mode"] == "attack"
            assert factory["values"]["goalkeeper.kick.type"] == "default"
            assert factory["values"]["goalkeeper.prediction.enabled"] is False
            status, telemetry = request(base, "/api/telemetry?limit=10")
            assert status == 200
            assert telemetry["events"][0]["prediction_reason"] == "threat_detected"
            connection = http.client.HTTPConnection(*base, timeout=3)
            connection.request("GET", "/")
            page = connection.getresponse()
            assert page.status == 200
            page.read()
            connection.close()
            status, result = request(base, "/api/apply", {
                "values": {
                    "goalkeeper.kick.type": "visual",
                    "goalkeeper.prediction.enabled": True,
                },
                "persist": True,
            })
            assert status == 200 and result["ok"]
            saved = config_path.read_text(encoding="utf-8")
            assert 'type: "visual"' in saved
            assert "enabled: true" in saved

            status, result = request(base, "/api/apply", {
                "values": {
                    "goalkeeper.prediction.min_samples": 15,
                    "goalkeeper.prediction.max_samples": 10,
                }
            })
            assert status == 400 and "error" in result
        finally:
            httpd.shutdown()
            httpd.server_close()
            thread.join(timeout=2)
    print("goalkeeper_web_test passed")


if __name__ == "__main__":
    main()
