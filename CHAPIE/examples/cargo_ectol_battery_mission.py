"""
examples/cargo_ectol_battery_mission.py — Generic cargo eCTOL battery-electric mission.

Illustrative example using representative performance values for a small
electric cargo eCTOL aircraft. Intended to demonstrate CHAPIE usage.

Run directly:
    python cargo_ectol_battery_mission.py
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from CHAPIE import (
    MissionProfile, PhaseSpec,
    PropulsionSystem, BatteryConfig,
    solve_mission,
)
from CHAPIE.aircraft.cargo_ectol import CargoECTOL
from CHAPIE.constants import KG_TO_LBS
from CHAPIE.plotting import plot_weight, plot_soc, plot_power


# ── Mission profile ───────────────────────────────────────────────────────────

cargo_ectol_mission = MissionProfile(
    cruise_alt_ft = 5000.0,
    #                  name     dur_min  power_kW  dist_nm  source     rep_alt_ft
    pre_cruise_phases = [
        PhaseSpec('Taxi',          5.0,   18.00,    0.0,  'battery',    0.0),
        PhaseSpec('Takeoff',       1.0,  300.00,    0.8,  'battery',    0.0),
        PhaseSpec('Climb',         7.0,  280.00,   14.0,  'battery', 2500.0),
    ],
    post_cruise_phases = [
        PhaseSpec('Descent',       4.0,    2.00,    4.0,  'battery', 2500.0),
        PhaseSpec('Approach',      2.0,   15.00,    3.0,  'battery',    0.0),
        PhaseSpec('Landing',       0.0,    0.00,    0.5,  'battery',    0.0),
    ],
)


# ── Propulsion configuration ──────────────────────────────────────────────────

bat_5pack = BatteryConfig(
    mass_kg    = 1100.0,
    energy_kWh = 200.0,
    n_packs    = 5,
)

prop_battery = PropulsionSystem(battery=bat_5pack)   # h2=None → battery-electric


# ── Single mission run ────────────────────────────────────────────────────────

def run_single(payload_kg: float = 200.0, target_range_nm: float = None) -> None:
    """Run and print one mission result.

    target_range_nm : total mission range [nm].  None = fly to max range.
    """
    ac = CargoECTOL()

    result = solve_mission(
        aircraft         = ac,
        propulsion       = prop_battery,
        mission          = cargo_ectol_mission,
        h2_fuel_mass_kg  = 0.0,
        payload_kg       = payload_kg,
        target_range_nm  = target_range_nm,
    )

    mtow_ok = result.start_weight_lbs <= ac.mtow_lbs
    costs   = result.cost()

    print(f"\nCargo eCTOL  |  Battery-electric  |  Payload: {payload_kg:.1f} kg  "
          f"({payload_kg * KG_TO_LBS:.0f} lbs)")
    print(f"  Take-off weight  : {result.start_weight_lbs:.1f} lbs  "
          f"({'OK' if mtow_ok else 'OVER MTOW'})")
    print(f"  Total range      : {result.total_range_nm:.1f} nm")
    print(f"  Cruise range     : {result.cruise_range_nm:.1f} nm")
    print(f"  Total time       : {result.total_time_h * 60:.1f} min")
    print(f"  Battery SOC end  : {result.battery_soc_final * 100:.1f}%")
    print(f"  Energy drawn     : {result.bat_net_energy_kWh:.2f} kWh")
    print(f"  Specific range   : {result.specific_range_nm_per_kWh:.3f} nm/kWh")
    print(f"  Mission cost     : ${costs['total_cost_usd']:.2f}")

    plot_weight(result)
    plot_soc(result, prop_battery)
    plot_power(result, ac, cargo_ectol_mission, prop_battery)


# ── Range-payload trade study ─────────────────────────────────────────────────

def range_payload_sweep() -> None:
    """Sweep payload from 0 to 500 lbs and print range at each point."""
    ac = CargoECTOL()

    print(f"\nRange-Payload trade  |  Cargo eCTOL Battery-electric  "
          f"|  Battery: {bat_5pack.energy_kWh:.0f} kWh")
    print(f"  {'Payload [lbs]':>15}  {'Range [nm]':>12}  "
          f"{'TOGW [lbs]':>12}  {'MTOW':>6}")
    print(f"  {'-'*51}")

    for payload_lbs in range(0, 600, 100):
        payload_kg = payload_lbs / KG_TO_LBS
        res = solve_mission(
            aircraft        = ac,
            propulsion      = prop_battery,
            mission         = cargo_ectol_mission,
            h2_fuel_mass_kg = 0.0,
            payload_kg      = payload_kg,
        )
        mtow_ok = res.start_weight_lbs <= ac.mtow_lbs
        print(f"  {payload_lbs:>15}  {res.total_range_nm:>12.1f}  "
              f"{res.start_weight_lbs:>12.1f}  {'OK' if mtow_ok else 'OVER':>6}")


if __name__ == '__main__':
    run_single()
    range_payload_sweep()
