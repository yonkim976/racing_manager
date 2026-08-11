"""Patch-safe module alias for the relocated FULL collision model."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.collision")
sys.modules[__name__] = _runtime_module
