"""
aircraft/base.py — Abstract base class for aircraft airframe definitions.

Subclass AircraftBase to integrate any fixed-wing or eCTOL aircraft
into CHAPIE.  The airframe defines aerodynamic performance but carries no
knowledge of the propulsion system or mission profile — those are supplied
separately to solve_mission().

Power convention:    all power values are electrical power [kW] at the DC bus.
Airspeed convention: cruise_speed_kts returns CAS; the solver converts to TAS
                     using atmosphere.cas_to_tas and the mission cruise altitude.
Mass convention:     all masses in kg; weights in lbs where explicitly named.
"""
from abc import ABC, abstractmethod


class AircraftBase(ABC):
    """Interface that all CHAPIE aircraft models must implement."""

    # ── Required properties ──────────────────────────────────────────────────

    @property
    @abstractmethod
    def empty_mass_kg(self) -> float:
        """
        Operational empty mass [kg].
        Excludes payload, propulsion system, and fuel.
        """

    @property
    @abstractmethod
    def mtow_lbs(self) -> float:
        """Maximum structural take-off weight [lbs]."""

    # ── Required performance methods ─────────────────────────────────────────

    @abstractmethod
    def cruise_power_kW(self, mass_kg: float) -> float:
        """
        Electrical power demand at the DC bus for steady level cruise [kW].
        Called at every Euler integration step with the current aircraft mass.
        """

    @abstractmethod
    def cruise_speed_kts(self, mass_kg: float) -> float:
        """
        Calibrated airspeed for steady level cruise [kts CAS].
        Called at every Euler integration step with the current aircraft mass.
        """

    # ── Optional override ────────────────────────────────────────────────────

    def phase_power_kW(self, P_ref_kW: float, mass_kg: float) -> float:
        """
        Scale a fixed-phase reference power to the actual aircraft mass [kW].

        Default: no scaling — returns P_ref_kW unchanged.
        Override for aircraft that derive phase powers from the cruise P-W model.

        Example override (linear P-W scaling):
            return P_ref_kW * self.cruise_power_kW(mass_kg) / self._P_ref_kW_max
        """
        return P_ref_kW
