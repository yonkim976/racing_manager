"""Module alias for relocated FULL trajectory physics."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.trajectory_physics")
sys.modules[__name__] = _runtime_module
