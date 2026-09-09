"""
CHAPIE — Conceptual Hybrid Aircraft Propulsion Integration Environment

A modular, optimizer-friendly framework for mission-level performance analysis
of hybrid and battery-electric propulsion systems.

Typical usage
-------------
from CHAPIE import (
    AircraftBase,
    MissionProfile, PhaseSpec,
    PropulsionSystem, BatteryConfig, H2Config,
    solve_mission,
    MissionResult,
)
"""

from .results              import MissionResult
from .aircraft.base        import AircraftBase
from .mission.profile      import MissionProfile, PhaseSpec
from .mission.solver       import solve_mission
from .propulsion.battery   import BatteryConfig
from .propulsion.fuel_cell import H2Config
from .propulsion.system    import PropulsionSystem

__version__ = '0.1.0'
__author__  = 'Chad Hess'
