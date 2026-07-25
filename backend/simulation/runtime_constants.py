"""Tiny cross-mixin runtime constants.

Domain mixins must not import the ``race_engine`` facade.  Constants that are
shared across domains and would otherwise create mixin cycles live here.
"""

from __future__ import annotations

# Start-lane merge distance also consulted by racecraft overtake gates.
GRID_LAUNCH_MERGE_DISTANCE_M = 135.0
