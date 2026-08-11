"""Patch-safe module alias for relocated FULL timing operations."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.timing_ops")
sys.modules[__name__] = _runtime_module
