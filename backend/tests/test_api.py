"""API-level tests for lightweight validation endpoints."""

from __future__ import annotations

import unittest
import os
from unittest.mock import patch

from starlette.requests import Request

from data_loader import load_circuits
from main import desktop_diagnostics, validate_circuit


class CircuitValidationApiTests(unittest.TestCase):
    def test_validate_circuit_accepts_compilable_seed_circuit(self) -> None:
        circuit = load_circuits()[0]

        response = validate_circuit(circuit)

        self.assertTrue(response["ok"])
        self.assertEqual(response["errors"], [])
        self.assertGreater(len(response["circuit"].track_coords), 20)


class DesktopDiagnosticsApiTests(unittest.TestCase):
    @staticmethod
    def _request(token: str = "") -> Request:
        return Request({
            "type": "http",
            "method": "GET",
            "path": "/api/desktop/diagnostics",
            "headers": [(b"x-f1-desktop-token", token.encode())],
            "query_string": b"",
            "server": ("127.0.0.1", 1),
            "client": ("127.0.0.1", 2),
            "scheme": "http",
        })

    def test_diagnostics_requires_desktop_mode_and_token(self) -> None:
        with patch.dict(os.environ, {"F1_DESKTOP_MODE": "1", "F1_DESKTOP_TOKEN": "secret"}):
            with self.assertRaisesRegex(Exception, "Desktop diagnostics token required"):
                desktop_diagnostics(self._request("wrong"))
            payload = desktop_diagnostics(self._request("secret"))

        self.assertEqual(payload["schema_version"], 1)
        self.assertEqual(payload["tire_temperature"]["schema_version"], 2)
        self.assertFalse(payload["active_session"])
        self.assertIn("rss_bytes", payload["process"])
        self.assertIn("cpu_percent", payload["process"])
        self.assertEqual(payload["safety_car"]["drivers"], [])
        self.assertEqual(
            payload["tire_temperature"]["thresholds"]["rear_surface_overheat_c"],
            130.0,
        )
        self.assertIn("peak", payload["tire_temperature"])
        self.assertNotIn("positions", payload)

    def test_diagnostics_is_disabled_in_normal_web_mode(self) -> None:
        with patch.dict(os.environ, {"F1_DESKTOP_MODE": "0", "F1_DESKTOP_TOKEN": "secret"}):
            with self.assertRaisesRegex(Exception, "Desktop diagnostics are disabled"):
                desktop_diagnostics(self._request("secret"))


if __name__ == "__main__":
    unittest.main()
