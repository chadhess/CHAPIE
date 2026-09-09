"""
aircraft/cargo_ectol.py — Generic cargo eCTOL aircraft airframe definition.

Illustrative example implementation of AircraftBase. Performance values are
representative of a small electric cargo eCTOL aircraft and are intended for
demonstration purposes only.

Cruise P-W and V-W models are linearly interpolated from two reference gross
weights at 5000 ft MSL. Phase powers scale proportionally to the cruise power
at the phase entry mass.
"""
from scipy.interpolate import interp1d

from .base import AircraftBase
from ..constants import KG_TO_LBS


class CargoECTOL(AircraftBase):
    """Generic cargo eCTOL aircraft airframe."""

    _EMPTY_MASS_KG = 1320.0              # operational empty mass [kg]

    _W_ref_lbs = [5000.0, 6000.0]        # reference gross weights [lbs]
    _P_ref_kW  = [95.0,   122.0]         # electrical cruise power at DC bus [kW]
    _V_ref_kts = [98.0,   108.0]         # calibrated cruise airspeed [kts CAS]

    @property
    def empty_mass_kg(self) -> float:
        return self._EMPTY_MASS_KG

    @property
    def mtow_lbs(self) -> float:
        return 6500.0

    def cruise_power_kW(self, mass_kg: float) -> float:
        """Interpolate electrical cruise power demand [kW] vs. gross mass."""
        f = interp1d(self._W_ref_lbs, self._P_ref_kW,
                     kind='linear', fill_value='extrapolate')
        return float(f(mass_kg * KG_TO_LBS))

    def cruise_speed_kts(self, mass_kg: float) -> float:
        """Interpolate calibrated cruise airspeed [kts CAS] vs. gross mass."""
        f = interp1d(self._W_ref_lbs, self._V_ref_kts,
                     kind='linear', fill_value='extrapolate')
        return float(f(mass_kg * KG_TO_LBS))

    def phase_power_kW(self, P_ref_kW: float, mass_kg: float) -> float:
        """Scale reference phase power proportionally to the cruise P-W model."""
        return P_ref_kW * self.cruise_power_kW(mass_kg) / self._P_ref_kW[-1]
