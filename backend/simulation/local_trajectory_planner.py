"""Patch-safe module alias for the relocated FULL local trajectory planner."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.local_trajectory_planner")
sys.modules[__name__] = _runtime_module
