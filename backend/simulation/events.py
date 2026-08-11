"""Patch-safe module alias for relocated FULL random race events."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.events")
sys.modules[__name__] = _runtime_module
