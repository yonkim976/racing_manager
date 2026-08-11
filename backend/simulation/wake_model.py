"""Patch-safe module alias for the relocated FULL wake model."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.wake_model")
sys.modules[__name__] = _runtime_module
