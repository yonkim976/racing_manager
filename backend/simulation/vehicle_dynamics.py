"""Module alias for relocated FULL vehicle force calculations."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.vehicle_dynamics")
sys.modules[__name__] = _runtime_module
