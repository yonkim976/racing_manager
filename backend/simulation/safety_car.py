"""Patch-safe module alias for relocated FULL Safety Car operations."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.safety_car")
sys.modules[__name__] = _runtime_module
