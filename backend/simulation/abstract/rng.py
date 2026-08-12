"""Patch-safe alias for ABSTRACT runtime rng."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.abstract.runtime.rng")
sys.modules[__name__] = _runtime_module
