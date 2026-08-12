"""Patch-safe alias for ABSTRACT runtime state."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.abstract.runtime.state")
sys.modules[__name__] = _runtime_module
