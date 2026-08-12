"""Patch-safe alias for ABSTRACT runtime progress_race."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.abstract.runtime.progress_race")
sys.modules[__name__] = _runtime_module
