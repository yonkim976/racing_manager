"""Module alias for relocated FULL track-surface physics."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.track_surface")
sys.modules[__name__] = _runtime_module
