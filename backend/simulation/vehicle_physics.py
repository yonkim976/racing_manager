"""Patch-safe module alias for relocated FULL vehicle physics."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.vehicle_physics")
sys.modules[__name__] = _runtime_module
