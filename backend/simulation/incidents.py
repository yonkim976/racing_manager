"""Patch-safe module alias for the relocated FULL incident model."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.incidents")
sys.modules[__name__] = _runtime_module
