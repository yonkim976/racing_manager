"""Module alias for the relocated FULL planner scheduler."""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.planner_scheduler")
sys.modules[__name__] = _runtime_module
