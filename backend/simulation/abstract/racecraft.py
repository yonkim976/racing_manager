"""Patch-safe alias for ABSTRACT runtime racecraft."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.abstract.runtime.racecraft")
sys.modules[__name__] = _runtime_module
