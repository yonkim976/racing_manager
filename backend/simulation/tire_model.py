"""Patch-safe module alias for relocated FULL tire calculations."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.tire_model")
sys.modules[__name__] = _runtime_module
