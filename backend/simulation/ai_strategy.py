"""Patch-safe module alias for relocated FULL AI strategy."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.ai_strategy")
sys.modules[__name__] = _runtime_module
