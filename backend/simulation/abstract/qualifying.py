"""Patch-safe alias for ABSTRACT runtime qualifying."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.abstract.runtime.qualifying")
sys.modules[__name__] = _runtime_module
