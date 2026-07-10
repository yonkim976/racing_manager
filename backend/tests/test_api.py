"""API-level tests for lightweight validation endpoints."""

from __future__ import annotations

import unittest

from data_loader import load_circuits
from main import validate_circuit


class CircuitValidationApiTests(unittest.TestCase):
    def test_validate_circuit_accepts_compilable_seed_circuit(self) -> None:
        circuit = load_circuits()[0]

        response = validate_circuit(circuit)

        self.assertTrue(response["ok"])
        self.assertEqual(response["errors"], [])
        self.assertGreater(len(response["circuit"].track_coords), 20)


if __name__ == "__main__":
    unittest.main()
