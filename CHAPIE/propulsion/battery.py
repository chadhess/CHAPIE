"""
propulsion/battery.py — Battery pack configuration.
"""
from dataclasses import dataclass


@dataclass
class BatteryConfig:
    """
    Battery pack configuration for one or more packs in series.

    energy_kWh and mass_kg describe the total system (all packs combined).
    """
    mass_kg:       float
    energy_kWh:    float
    n_packs:       int   = 1
    eta_charge:    float = 0.995   # coulombic efficiency when charging
    eta_discharge: float = 0.995   # coulombic efficiency when discharging

    @property
    def mass_per_pack_kg(self) -> float:
        return self.mass_kg / self.n_packs

    @property
    def energy_per_pack_kWh(self) -> float:
        return self.energy_kWh / self.n_packs

    @property
    def specific_energy_kWh_per_kg(self) -> float:
        return self.energy_kWh / self.mass_kg
