"""Patch-safe module alias for the relocated FULL brake model."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.brake_model")
sys.modules[__name__] = _runtime_module
