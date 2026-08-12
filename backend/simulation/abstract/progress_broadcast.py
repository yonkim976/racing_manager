"""Patch-safe alias for ABSTRACT runtime progress_broadcast."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.abstract.runtime.progress_broadcast")
sys.modules[__name__] = _runtime_module
