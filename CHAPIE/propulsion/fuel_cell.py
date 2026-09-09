"""
propulsion/fuel_cell.py — Hydrogen storage and fuel cell system configuration.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Tuple

from ..constants import H2_MOLAR_MASS_kg, R_UNIVERSAL


@dataclass
class H2Config:
    """
    Hydrogen storage and fuel cell system configuration.

    Parameters
    ----------
    pressure_bar    : tank storage pressure [bar]
    Z_factor        : compressibility factor Z for real-gas H2 density
    grav_eff        : tank gravimetric efficiency = m_fuel / m_tank_system
    fc_max_power_kw : FC maximum net DC output at sea level [kW].
                      Used at all altitudes when alt_derating is empty.
    fc_mass_kg      : fuel cell system mass (stack + BOP hardware) [kg]
    eta_fc          : fuel cell stack electrochemical efficiency
                      (H2 LHV chemical energy → stack gross electrical output)
    eta_BOP         : balance of plant efficiency — fraction of stack gross
                      output remaining after BOP parasitic loads
    tank_temp_K     : H2 storage temperature [K]
    pack_eff        : cylinder packing efficiency within bay volume
    fill_frac       : usable fill fraction of each cylinder bore
    alt_derating    : altitude derating table for FC net DC output as
                      [(alt_ft, max_kw), ...].  Points need not be pre-sorted.
                      If empty, fc_max_power_kw is used at all altitudes.

    Notes
    -----
    DC bus power convention: fc_max_power_kw and alt_derating values are
    net electrical output at the FC system terminals (post-BOP), which is
    the power available to the aircraft's DC bus.

    H2 consumption for a given DC bus demand P_dc:
        H2 rate [kg/h] = P_dc / (eta_fc * eta_BOP * H2_LHV_kWh_per_kg)
    """

    pressure_bar:    float
    Z_factor:        float
    grav_eff:        float
    fc_max_power_kw: float                      = 200.0
    fc_mass_kg:      float                      = 0.0
    eta_fc:          float                      = 0.50
    eta_BOP:         float                      = 0.95
    tank_temp_K:     float                      = 293.0
    pack_eff:        float                      = 0.785
    fill_frac:       float                      = 0.85
    alt_derating:    List[Tuple[float, float]]  = field(default_factory=list)

    def __post_init__(self):
        if self.alt_derating:
            self.alt_derating = sorted(self.alt_derating, key=lambda p: p[0])

    # ── FC power ─────────────────────────────────────────────────────────────

    def fc_max_at_alt_kw(self, alt_ft: float) -> float:
        """
        FC maximum net DC output derated for altitude [kW].

        Linearly interpolates the alt_derating table; clamps at boundaries.
        Returns fc_max_power_kw if no derating table is provided.
        """
        if not self.alt_derating:
            return self.fc_max_power_kw
        alts = [p[0] for p in self.alt_derating]
        pows = [p[1] for p in self.alt_derating]
        if alt_ft <= alts[0]:
            return pows[0]
        if alt_ft >= alts[-1]:
            return pows[-1]
        for i in range(len(alts) - 1):
            if alts[i] <= alt_ft <= alts[i + 1]:
                t = (alt_ft - alts[i]) / (alts[i + 1] - alts[i])
                return pows[i] + t * (pows[i + 1] - pows[i])
        return pows[-1]

    # ── Tank geometry and fuel mass ───────────────────────────────────────────

    @property
    def rho_kg_m3(self) -> float:
        """Real-gas H2 density at storage conditions [kg/m³]."""
        return (self.pressure_bar * 1e5 * H2_MOLAR_MASS_kg) / (
            R_UNIVERSAL * self.tank_temp_K * self.Z_factor
        )

    def usable_gas_volume_m3(self, bay_vol_m3: float, n_bays: int) -> float:
        """H2 gas volume after cylinder packing and fill losses [m³]."""
        return n_bays * bay_vol_m3 * self.pack_eff * self.fill_frac

    def fuel_mass_kg(self, bay_vol_m3: float, n_bays: int) -> float:
        """H2 fuel mass that fits in the given bay geometry [kg]."""
        return self.rho_kg_m3 * self.usable_gas_volume_m3(bay_vol_m3, n_bays)

    def tank_structure_mass_kg(self, fuel_mass_kg: float) -> float:
        """Tank structural mass for a given H2 fuel load [kg]."""
        return fuel_mass_kg * (1.0 / self.grav_eff - 1.0)
