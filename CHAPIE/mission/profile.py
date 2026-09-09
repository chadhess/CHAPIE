"""
mission/profile.py — Mission profile definition, independent of the airframe.

Separating the mission from the aircraft enables:
  - Same airframe evaluated across multiple mission types
  - Parameterised phase durations and powers for optimisation loops
  - Clean reuse of phase tables across different propulsion configurations
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, NamedTuple


class PhaseSpec(NamedTuple):
    """
    Specification for one fixed-duration flight phase.

    Attributes
    ----------
    name        : human-readable phase label (e.g. 'Climb', 'Approach')
    dur_min     : phase duration [minutes]; set to 0.0 to skip
    power_kW    : reference electrical power at the DC bus [kW].
                  Scaled to actual aircraft mass by aircraft.phase_power_kW()
                  if that method is overridden; otherwise used directly.
    dist_nm     : ground distance covered [nm]; use 0.0 for hover/ground phases
    source      : energy source for this phase —
                    'h2'      : FC is primary; buffer battery covers any deficit
                    'battery' : battery only; FC not active
    rep_alt_ft  : representative altitude for FC altitude derating [ft].
                  Use the average altitude for climb/descent phases
                  (e.g. 2500 ft for a 0→5000 ft climb).
    """
    name:       str
    dur_min:    float
    power_kW:   float
    dist_nm:    float
    source:     str
    rep_alt_ft: float


@dataclass
class MissionProfile:
    """
    Complete mission profile: fixed phases flanking an energy-limited cruise.

    The solver executes in order:
        pre_cruise_phases → cruise (H2 or battery limited) → post_cruise_phases

    Parameters
    ----------
    cruise_alt_ft      : cruise altitude above MSL [ft].
                         Used for CAS→TAS conversion and FC altitude derating
                         during the cruise segment.
    pre_cruise_phases  : phases before cruise (e.g. taxi, takeoff, climb)
    post_cruise_phases : phases after cruise (e.g. descent, approach, landing)
    """
    cruise_alt_ft:      float
    pre_cruise_phases:  List[PhaseSpec] = field(default_factory=list)
    post_cruise_phases: List[PhaseSpec] = field(default_factory=list)

    def total_fixed_range_nm(self) -> float:
        """Total ground distance covered in all fixed phases [nm]."""
        return sum(p.dist_nm for p in self.pre_cruise_phases + self.post_cruise_phases)

    def total_fixed_time_min(self) -> float:
        """Total duration of all fixed phases [min]."""
        return sum(p.dur_min for p in self.pre_cruise_phases + self.post_cruise_phases)
