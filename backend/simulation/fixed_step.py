"""Patch-safe module alias for the relocated FULL fixed-step accumulator."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.fixed_step")
sys.modules[__name__] = _runtime_module
