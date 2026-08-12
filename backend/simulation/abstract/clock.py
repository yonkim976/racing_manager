"""Patch-safe alias for ABSTRACT runtime clock."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.abstract.runtime.clock")
sys.modules[__name__] = _runtime_module
