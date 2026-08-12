"""Patch-safe alias for ABSTRACT runtime pose."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.abstract.runtime.pose")
sys.modules[__name__] = _runtime_module
