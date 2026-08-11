"""Module alias for relocated FULL state contracts."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.state_contract")
sys.modules[__name__] = _runtime_module
