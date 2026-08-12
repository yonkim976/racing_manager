"""Module alias for relocated FULL constructor performance calculations."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.car_performance")
sys.modules[__name__] = _runtime_module
