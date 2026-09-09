"""
results.py — MissionResult dataclass and post-processing helpers.

All energies in kWh, masses in kg, distances in nautical miles, times in hours.
"""
from dataclasses import dataclass, field
from typing import List, Tuple

from .constants import H2_LHV_kWh_per_kg, KG_TO_LBS


@dataclass
class MissionResult:
    """Output of a single solve_mission() call."""

    total_range_nm:     float
    cruise_range_nm:    float
    total_time_h:       float
    cruise_time_h:      float
    h2_burned_kg:       float
    bat_net_energy_kWh: float        # battery energy drawn net of recharge [kWh]
    battery_soc_final:  float        # 0.0–1.0
    start_mass_kg:      float
    end_mass_kg:        float
    payload_kg:         float
    mass_vs_time:       List[Tuple[float, float]] = field(default_factory=list)  # (t_h, kg)
    bat_soc_vs_time:    List[Tuple[float, float]] = field(default_factory=list)  # (t_h, 0–1)

    # ── Derived properties ───────────────────────────────────────────────────

    @property
    def start_weight_lbs(self) -> float:
        """Takeoff gross weight [lbs]."""
        return self.start_mass_kg * KG_TO_LBS

    @property
    def h2_energy_kWh(self) -> float:
        """Total H2 energy consumed [kWh LHV]."""
        return self.h2_burned_kg * H2_LHV_kWh_per_kg

    @property
    def range_per_kg_h2_nm(self) -> float:
        """Range efficiency [nm / kg H2]; inf for battery-only missions."""
        if self.h2_burned_kg < 1e-9:
            return float('inf')
        return self.total_range_nm / self.h2_burned_kg

    @property
    def specific_range_nm_per_kWh(self) -> float:
        """Specific range [nm / kWh total energy consumed]."""
        total_e = self.h2_energy_kWh + self.bat_net_energy_kWh
        return self.total_range_nm / total_e if total_e > 0 else 0.0

    def cost(
        self,
        h2_price_usd_per_kg:    float = 10.0,
        elec_price_usd_per_kWh: float = 0.15,
    ) -> dict:
        """Estimated mission fuel/energy cost breakdown."""
        h2_cost  = self.h2_burned_kg * h2_price_usd_per_kg
        bat_cost = self.bat_net_energy_kWh * elec_price_usd_per_kWh
        return {
            'h2_burned_kg':   self.h2_burned_kg,
            'h2_energy_kWh':  self.h2_energy_kWh,
            'bat_energy_kWh': self.bat_net_energy_kWh,
            'h2_cost_usd':    h2_cost,
            'bat_cost_usd':   bat_cost,
            'total_cost_usd': h2_cost + bat_cost,
        }
