"""Module alias for the relocated FULL race engine.

The alias intentionally exposes the runtime module object itself rather than a
symbol copy. Existing integrations that patch ``simulation.race_engine``
therefore still patch the globals used by :class:`RaceEngine`.
"""

from importlib import import_module
import sys


_runtime_module = import_module("engines.full.runtime.race_engine")
sys.modules[__name__] = _runtime_module
