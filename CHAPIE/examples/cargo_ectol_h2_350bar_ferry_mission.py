"""
examples/cargo_ectol_h2_350bar_ferry_mission.py — Generic cargo eCTOL H2 ferry mission (350 bar).

Illustrative ferry configuration: 4 dedicated H2 bays plus remaining cargo bay
space (FC bounding box subtracted). Payload is zero — cargo bay is occupied by
the additional H2 tank. Intended to demonstrate CHAPIE usage.

Cargo bay available volume:
    CARGO_TANK_SPACE_M3 = cargo_bay_vol × usable_frac − FC_bounding_box_vol
                        = 4.5 × 0.75 − (1.0 × 0.8 × 0.7)
                        ≈ 2.815 m³

Run directly:
    python cargo_ectol_h2_350bar_ferry_mission.py
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from CHAPIE import (
    MissionProfile, PhaseSpec,
    PropulsionSystem, BatteryConfig, H2Config,
    solve_mission,
)
from CHAPIE.aircraft.cargo_ectol import CargoECTOL
from CHAPIE.plotting import plot_weight, plot_soc, plot_power


# ── Mission profile ───────────────────────────────────────────────────────────

cargo_ectol_mission = MissionProfile(
    cruise_alt_ft = 5000.0,
    #                  name     dur_min  power_kW  dist_nm  source     rep_alt_ft
    pre_cruise_phases = [
        PhaseSpec('Taxi',          5.0,   18.00,    0.0,  'h2',         0.0),
        PhaseSpec('Takeoff',       1.0,  300.00,    0.8,  'h2',         0.0),
        PhaseSpec('Climb',         7.0,  280.00,   14.0,  'h2',      2500.0),
    ],
    post_cruise_phases = [
        PhaseSpec('Descent',       4.0,    2.00,    4.0,  'battery', 2500.0),
        PhaseSpec('Approach',      2.0,   15.00,    3.0,  'battery',    0.0),
        PhaseSpec('Landing',       0.0,    0.00,    0.5,  'battery',    0.0),
    ],
)


# ── Propulsion configuration ──────────────────────────────────────────────────

FC_DERATING = [
    (0,     150.0),
    (3500,  145.0),
    (6000,  140.0),
    (10000, 128.0),
]

h2_350bar = H2Config(
    pressure_bar    = 350,
    Z_factor        = 1.20,
    grav_eff        = 0.080,
    fc_max_power_kw = 150.0,
    fc_mass_kg      = 200.0,
    eta_fc          = 0.50,
    eta_BOP         = 0.95,
    alt_derating    = FC_DERATING,
)

buffer_battery = BatteryConfig(
    mass_kg    = 200.0,
    energy_kWh = 35.0,
    n_packs    = 1,
)

propulsion_350 = PropulsionSystem(
    battery         = buffer_battery,
    h2              = h2_350bar,
    cooling_mass_kg = 15.0,
    mounts_mass_kg  = 15.0,
)


# ── Storage geometry ──────────────────────────────────────────────────────────

BAY_VOL_M3 = 1.0 * 0.6 * 0.3              # per dedicated H2 bay [m³]
N_BAYS     = 4                             # dedicated H2 bays

# Cargo bay: total volume × usable fraction − FC bounding box
CARGO_TANK_SPACE_M3 = 4.5 * 0.75 - (1.0 * 0.8 * 0.7)


# ── Ferry fuel load ───────────────────────────────────────────────────────────

def ferry_fuel_kg() -> float:
    """Total H2 for ferry: 4 dedicated bays + remaining cargo bay space."""
    fuel_bays  = h2_350bar.fuel_mass_kg(BAY_VOL_M3, N_BAYS)
    fuel_cargo = h2_350bar.fuel_mass_kg(CARGO_TANK_SPACE_M3, 1)
    return fuel_bays + fuel_cargo


# ── Single mission run ────────────────────────────────────────────────────────

def run_single(target_range_nm: float = None) -> None:
    """Run and print one ferry mission result.

    target_range_nm : total mission range [nm].  None = fly to max range.
    """
    ac      = CargoECTOL()
    fuel_kg = ferry_fuel_kg()

    result = solve_mission(
        aircraft         = ac,
        propulsion       = propulsion_350,
        mission          = cargo_ectol_mission,
        h2_fuel_mass_kg  = fuel_kg,
        payload_kg       = 0.0,
        target_range_nm  = target_range_nm,
    )

    mtow_ok = result.start_weight_lbs <= ac.mtow_lbs
    costs   = result.cost()

    print("\nCargo eCTOL  |  350 bar H2 Ferry  |  Payload: 0 kg")
    print(f"  H2 fuel loaded   : {fuel_kg:.2f} kg")
    print(f"    - {N_BAYS} bays      : {h2_350bar.fuel_mass_kg(BAY_VOL_M3, N_BAYS):.2f} kg")
    print(f"    - cargo bay    : {h2_350bar.fuel_mass_kg(CARGO_TANK_SPACE_M3, 1):.2f} kg")
    print(f"    - cargo vol    : {CARGO_TANK_SPACE_M3:.3f} m³")
    print(f"  Take-off weight  : {result.start_weight_lbs:.1f} lbs  "
          f"({'OK' if mtow_ok else 'OVER MTOW'})")
    print(f"  Total range      : {result.total_range_nm:.1f} nm")
    print(f"  Cruise range     : {result.cruise_range_nm:.1f} nm")
    print(f"  Total time       : {result.total_time_h * 60:.1f} min")
    print(f"  H2 burned        : {result.h2_burned_kg:.2f} kg")
    print(f"  Range / kg H2    : {result.range_per_kg_h2_nm:.2f} nm/kg")
    print(f"  Battery SOC end  : {result.battery_soc_final * 100:.1f}%")
    print(f"  Mission cost     : ${costs['total_cost_usd']:.2f}")

    plot_weight(result)
    plot_soc(result, propulsion_350)
    plot_power(result, ac, cargo_ectol_mission, propulsion_350)


if __name__ == '__main__':
    run_single()
