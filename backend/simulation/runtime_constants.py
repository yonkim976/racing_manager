"""Module alias for relocated FULL runtime constants."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.runtime_constants")
sys.modules[__name__] = _runtime_module
