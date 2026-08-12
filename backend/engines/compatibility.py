"""Versioned compatibility contract for relocated engine modules.

Runtime authority lives under :mod:`engines.full.runtime` and
:mod:`engines.abstract.runtime`.  The legacy paths below remain importable for
external scripts and previously packaged integrations, but new repository code
must not use them as authority imports.
"""

from __future__ import annotations

from types import MappingProxyType


ENGINE_COMPATIBILITY_POLICY_VERSION = "engine-runtime-compat-v1"

FULL_PATCH_SAFE_MODULE_ALIASES = MappingProxyType({
    "simulation.ai_strategy": "engines.full.runtime.ai_strategy",
    "simulation.brake_model": "engines.full.runtime.brake_model",
    "simulation.car_performance": "engines.full.runtime.car_performance",
    "simulation.collision": "engines.full.runtime.collision",
    "simulation.events": "engines.full.runtime.events",
    "simulation.fixed_step": "engines.full.runtime.fixed_step",
    "simulation.incident_ops": "engines.full.runtime.incident_ops",
    "simulation.incidents": "engines.full.runtime.incidents",
    "simulation.local_trajectory_planner": (
        "engines.full.runtime.local_trajectory_planner"
    ),
    "simulation.physics": "engines.full.runtime.physics",
    "simulation.pit_ops": "engines.full.runtime.pit_ops",
    "simulation.pit_stop": "engines.full.runtime.pit_stop",
    "simulation.race_engine": "engines.full.runtime.race_engine",
    "simulation.racecraft_ops": "engines.full.runtime.racecraft_ops",
    "simulation.planner_scheduler": "engines.full.runtime.planner_scheduler",
    "simulation.runtime_constants": "engines.full.runtime.runtime_constants",
    "simulation.safety_car": "engines.full.runtime.safety_car",
    "simulation.speed_profile": "engines.full.runtime.speed_profile",
    "simulation.start_ops": "engines.full.runtime.start_ops",
    "simulation.state_contract": "engines.full.runtime.state_contract",
    "simulation.strategy_ops": "engines.full.runtime.strategy_ops",
    "simulation.timing_ops": "engines.full.runtime.timing_ops",
    "simulation.tire_model": "engines.full.runtime.tire_model",
    "simulation.track_surface": "engines.full.runtime.track_surface",
    "simulation.trajectory_physics": "engines.full.runtime.trajectory_physics",
    "simulation.vehicle_dynamics": "engines.full.runtime.vehicle_dynamics",
    "simulation.vehicle_physics": "engines.full.runtime.vehicle_physics",
    "simulation.wake_model": "engines.full.runtime.wake_model",
})

ABSTRACT_PATCH_SAFE_MODULE_ALIASES = MappingProxyType({
    f"simulation.abstract.{module_name}": (
        f"engines.abstract.runtime.{module_name}"
    )
    for module_name in (
        "broadcast",
        "clock",
        "engine",
        "incidents",
        "kinematics",
        "performance",
        "pit",
        "pose",
        "progress_broadcast",
        "progress_race",
        "qualifying",
        "race",
        "racecraft",
        "replay",
        "rng",
        "state",
    )
})

# These facades preserve public symbols but intentionally are not module-object
# aliases.  Their symbol identity is covered by engine-boundary tests.
SYMBOL_COMPATIBILITY_FACADES = MappingProxyType({
    "simulation.qualifying": "engines.full.runtime.qualifying",
    "simulation.abstract": "engines.abstract.runtime",
})

ALL_PATCH_SAFE_MODULE_ALIASES = MappingProxyType({
    **FULL_PATCH_SAFE_MODULE_ALIASES,
    **ABSTRACT_PATCH_SAFE_MODULE_ALIASES,
})

COMPATIBILITY_REMOVAL_GATES = (
    "repository authority imports remain zero",
    "packaged-app smoke passes without compatibility imports",
    "external scripts and saved integrations have a documented migration path",
    "removal is approved as an explicit breaking change",
)


__all__ = [
    "ABSTRACT_PATCH_SAFE_MODULE_ALIASES",
    "ALL_PATCH_SAFE_MODULE_ALIASES",
    "COMPATIBILITY_REMOVAL_GATES",
    "ENGINE_COMPATIBILITY_POLICY_VERSION",
    "FULL_PATCH_SAFE_MODULE_ALIASES",
    "SYMBOL_COMPATIBILITY_FACADES",
]
