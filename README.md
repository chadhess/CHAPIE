# CHAPIE

**Conceptual Hybrid Aircraft Propulsion Integration Environment**

CHAPIE is a Python framework for mission-level performance analysis of hybrid-electric and battery-electric aircraft. Performance is tracked over a user-defined mission profile to compute range, energy consumption, hydrogen fuel burn, battery state of charge, and operating costs. Users have fully modular control across base aircraft, propulsion system, and mission definition.

Developed as part of advanced air mobility (AAM) research on hydrogen fuel cell integration into battery-electric aircraft.

---

## Installation

Clone the repository and install with pip:

```bash
git clone https://github.com/chadhess/CHAPIE.git
cd CHAPIE
pip install .
```

Or install directly from GitHub:

```bash
pip install git+https://github.com/chadhess/CHAPIE.git
```

**Dependencies:** Python >= 3.9, NumPy, SciPy, Matplotlib

---

## Quickstart

```python
from CHAPIE import (
    MissionProfile, PhaseSpec,
    PropulsionSystem, BatteryConfig, H2Config,
    solve_mission,
)
from CHAPIE.aircraft.cx300 import CX300

# Define mission phases
mission = MissionProfile(
    cruise_alt_ft=5000.0,
    #                  name      dur_min  power_kW  dist_nm  source     rep_alt_ft
    pre_cruise_phases=[
        PhaseSpec('Taxi',          5.0,   20.40,    0.0,  'h2',         0.0),
        PhaseSpec('Takeoff',       0.7,  351.43,    0.7,  'h2',         0.0),
        PhaseSpec('Climb',         6.4,  303.75,   13.5,  'h2',      2500.0),
    ],
    post_cruise_phases=[
        PhaseSpec('Descent',       4.2,    1.43,    4.0,  'battery', 2500.0),
        PhaseSpec('Approach',      1.7,   17.65,    3.0,  'battery',    0.0),
        PhaseSpec('Landing',       0.0,    0.00,    0.6,  'battery',    0.0),
    ],
)

# Configure propulsion system (H2 hybrid)
h2 = H2Config(
    pressure_bar=700, Z_factor=1.45, grav_eff=0.065,
    fc_max_power_kw=188.0, fc_mass_kg=230.0,
    eta_fc=0.50, eta_BOP=0.95,
    alt_derating=[(0, 188.0), (3500, 183.0), (6000, 179.0), (10000, 163.0)],
)
battery = BatteryConfig(mass_kg=238.1, energy_kWh=43.8, n_packs=1)
propulsion = PropulsionSystem(battery=battery, h2=h2, cooling_mass_kg=20.0, mounts_mass_kg=20.0)

# Run mission
ac = CX300()
fuel_kg = h2.fuel_mass_kg(1.215 * 0.666 * 0.315, n_tanks=4)

result = solve_mission(
    aircraft=ac, propulsion=propulsion, mission=mission,
    h2_fuel_mass_kg=fuel_kg, payload_kg=453.6,
)

print(f"Total range  : {result.total_range_nm:.1f} nm")
print(f"H2 burned    : {result.h2_burned_kg:.2f} kg")
print(f"Battery SOC  : {result.battery_soc_final * 100:.1f}%")
```

See the `CHAPIE/examples/` directory for complete worked examples covering:
- Battery-electric mission (`cx300_battery_mission.py`)
- H2 hybrid at 350 bar and 700 bar storage (`cx300_h2_350bar_mission.py`, `cx300_h2_700bar_mission.py`)
- Ferry configurations with maximum fuel load (`cx300_h2_350bar_ferry_mission.py`, `cx300_h2_700bar_ferry_mission.py`)

---

## Key Features

- **Modular architecture** — aircraft, propulsion, and mission are defined independently and integrated at solve time
- **H2 hybrid and battery-electric power** — single solver handles both; propulsion mode is set using the `PropulsionSystem` configuration
- **Altitude-derated fuel cell model** — FC max power interpolated from a user-supplied derating table
- **Battery charging during cruise** — FC runs above demand to recharge buffer battery when headroom is available
- **Target range mode** — optionally terminate cruise at a specified total mission range rather than flying to energy exhaustion
- **Range-payload trade sweep** — built into example files to demonstrate propulsion system sizing based on payload requirements
- **Mission plotting** — weight, battery SOC, and power time histories via `CHAPIE.plotting`

---

## Power Convention

All power values throughout CHAPIE are **electrical power at the DC bus [kW]**. Downstream losses (inverter, motor, gearbox) are embedded in the aircraft's cruise power model. Upstream losses (FC electrochemistry, BOP efficiency, battery round-trip efficiency) are applied internally by the solver.

---

## Package Structure

```
CHAPIE/
├── aircraft/
│   ├── base.py          # AircraftBase abstract class
│   └── cx300.py         # CX300 eCTOL aircraft implementation
├── mission/
│   ├── profile.py       # MissionProfile and PhaseSpec
│   └── solver.py        # Euler-integration mission solver
├── propulsion/
│   ├── battery.py       # BatteryConfig
│   ├── fuel_cell.py     # H2Config
│   └── system.py        # PropulsionSystem
├── examples/            # Worked examples for the CX300
├── atmosphere.py        # ISA atmosphere model (CAS to TAS)
├── constants.py         # Physical constants
├── plotting.py          # Mission result visualisation
└── results.py           # MissionResult dataclass
```

---

## Adding a New Aircraft

Subclass `AircraftBase` and implement four items:

```python
from CHAPIE.aircraft.base import AircraftBase

class MyAircraft(AircraftBase):
    @property
    def empty_mass_kg(self) -> float:
        return 1200.0          # airframe OEW, excluding propulsion and payload

    @property
    def mtow_lbs(self) -> float:
        return 6500.0

    def cruise_power_kW(self, mass_kg: float) -> float:
        # electrical DC bus power for steady level cruise
        ...

    def cruise_speed_kts(self, mass_kg: float) -> float:
        # calibrated airspeed (CAS) for steady level cruise
        ...
```

---

## License

MIT License. See [LICENSE](LICENSE) for details.
