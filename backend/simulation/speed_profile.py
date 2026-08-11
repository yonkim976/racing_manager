"""Patch-safe module alias for the relocated FULL speed profile."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.speed_profile")
sys.modules[__name__] = _runtime_module
