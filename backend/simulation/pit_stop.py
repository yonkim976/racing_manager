"""Patch-safe module alias for relocated FULL pit-stop logic."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.pit_stop")
sys.modules[__name__] = _runtime_module
