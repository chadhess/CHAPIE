"""
propulsion/system.py — Combined propulsion system definition.
"""
from dataclasses import dataclass

from .battery   import BatteryConfig
from .fuel_cell import H2Config


@dataclass
class PropulsionSystem:
    """
    Complete propulsion system for one configuration.

    H2 hybrid:        battery is the buffer pack; h2 is the FC + tank config.
    Battery-electric: battery is the full energy storage; h2 is None.

    Parameters
    ----------
    battery         : primary energy storage (buffer pack for H2 hybrid;
                      full pack for battery-electric)
    h2              : H2 fuel cell + tank config; None for battery-electric
    cooling_mass_kg : thermal management system mass [kg]
    mounts_mass_kg  : structural mounts and safety hardware mass [kg]
    """
    battery:         BatteryConfig
    h2:              H2Config = None
    cooling_mass_kg: float    = 0.0
    mounts_mass_kg:  float    = 0.0

    @property
    def non_fuel_fixed_mass_kg(self) -> float:
        """
        Fixed system mass excluding H2 fuel and tank structure [kg].
        Constant throughout the mission (does not burn off).
        Includes: battery + FC system + cooling + mounts.
        """
        fc_mass = self.h2.fc_mass_kg if self.h2 is not None else 0.0
        return (self.battery.mass_kg
                + fc_mass
                + self.cooling_mass_kg
                + self.mounts_mass_kg)
