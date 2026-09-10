"""
examples/cargo_ectol_fc_optimization.py — FC sizing study for the cargo eCTOL (350 bar H2).

Sweeps FC maximum power to find the range-maximising design within the MTOW
constraint, using a fixed fuel load from the 4 dedicated H2 bays only (no
cargo bay overflow). A second sweep over cruise altitude produces a contour map
showing the combined effect of FC sizing and cruise altitude on mission range.

See cargo_ectol_fc_optimization_ferry.py for the equivalent study with maximum
fuel load (4 bays + cargo bay overflow, payload = 0).

Assumptions
-----------
FC specific power         : 0.75 kW/kg   (mass = P_fc / 0.75)
Altitude derating         : proportionally scaled from base 150 kW table
Climb rate                : 714 ft/min   (derived: 5000 ft / 7 min)
Climb speed               : 120 kts      (derived: 14 nm / 7 min)
Descent rate              : 1250 ft/min  (derived: 5000 ft / 4 min)
Descent speed             : 60 kts       (derived: 4 nm / 4 min)
Takeoff and climb powers  : held constant across altitude sweep
Payload                   : 200 kg (fixed for all runs)

Run directly:
    python cargo_ectol_fc_optimization.py
"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from CHAPIE import (
    MissionProfile, PhaseSpec,
    PropulsionSystem, BatteryConfig, H2Config,
    solve_mission,
)
from CHAPIE.aircraft.cargo_ectol import CargoECTOL
from CHAPIE.constants import KG_TO_LBS
from CHAPIE.plotting import plot_weight, plot_soc, plot_power


# ── Constants ─────────────────────────────────────────────────────────────────

FC_SPECIFIC_POWER_KW_PER_KG = 0.75    # kW/kg — given

FC_BASE_POWER_KW = 150.0
FC_BASE_DERATING = [
    (0,     150.0),
    (3500,  145.0),
    (6000,  140.0),
    (10000, 128.0),
]

H2_PRESSURE_BAR = 350
H2_Z_FACTOR     = 1.20
H2_GRAV_EFF     = 0.080
H2_ETA_FC       = 0.50
H2_ETA_BOP      = 0.95

BAY_VOL_M3 = 1.0 * 0.6 * 0.3   # per dedicated H2 bay [m³]
N_BAYS     = 4

BUFFER_BAT_MASS_KG    = 200.0
BUFFER_BAT_ENERGY_KWH = 35.0
COOLING_MASS_KG       = 15.0
MOUNTS_MASS_KG        = 15.0

TAXI_DUR_MIN      = 5.0;  TAXI_POWER_KW     = 18.0
TAKEOFF_DUR_MIN   = 1.0;  TAKEOFF_POWER_KW  = 300.0;  TAKEOFF_DIST_NM  = 0.8
CLIMB_POWER_KW    = 280.0
DESCENT_POWER_KW  = 2.0
APPROACH_DUR_MIN  = 2.0;  APPROACH_POWER_KW = 15.0;   APPROACH_DIST_NM = 3.0
LANDING_DIST_NM   = 0.5

# Climb / descent rates derived from base mission phases at 5000 ft
CLIMB_RATE_FPM    = 5000.0 / 7.0              # 714 ft/min
CLIMB_SPEED_KTS   = 14.0  / (7.0  / 60.0)    # 120 kts
DESCENT_RATE_FPM  = 5000.0 / 4.0             # 1250 ft/min
DESCENT_SPEED_KTS = 4.0   / (4.0  / 60.0)   # 60 kts

PAYLOAD_KG = 200.0


# ── Builders ──────────────────────────────────────────────────────────────────

def _build_h2_config(fc_power_kw: float) -> H2Config:
    scale    = fc_power_kw / FC_BASE_POWER_KW
    derating = [(alt, pwr * scale) for alt, pwr in FC_BASE_DERATING]
    return H2Config(
        pressure_bar    = H2_PRESSURE_BAR,
        Z_factor        = H2_Z_FACTOR,
        grav_eff        = H2_GRAV_EFF,
        fc_max_power_kw = fc_power_kw,
        fc_mass_kg      = fc_power_kw / FC_SPECIFIC_POWER_KW_PER_KG,
        eta_fc          = H2_ETA_FC,
        eta_BOP         = H2_ETA_BOP,
        alt_derating    = derating,
    )


def _build_propulsion(fc_power_kw: float) -> PropulsionSystem:
    return PropulsionSystem(
        battery         = BatteryConfig(mass_kg    = BUFFER_BAT_MASS_KG,
                                        energy_kWh = BUFFER_BAT_ENERGY_KWH,
                                        n_packs    = 1),
        h2              = _build_h2_config(fc_power_kw),
        cooling_mass_kg = COOLING_MASS_KG,
        mounts_mass_kg  = MOUNTS_MASS_KG,
    )


def _total_h2_fuel_kg(fc_power_kw: float, h2_cfg: H2Config) -> float:
    """Fuel from 4 dedicated bays only — cargo bay not used for H2."""
    return h2_cfg.fuel_mass_kg(BAY_VOL_M3, N_BAYS)


def _build_mission(cruise_alt_ft: float) -> MissionProfile:
    climb_min = cruise_alt_ft / CLIMB_RATE_FPM
    climb_nm  = CLIMB_SPEED_KTS  * (climb_min / 60.0)
    desc_min  = cruise_alt_ft / DESCENT_RATE_FPM
    desc_nm   = DESCENT_SPEED_KTS * (desc_min  / 60.0)
    mid_alt   = cruise_alt_ft / 2.0

    return MissionProfile(
        cruise_alt_ft     = cruise_alt_ft,
        pre_cruise_phases = [
            PhaseSpec('Taxi',    TAXI_DUR_MIN,   TAXI_POWER_KW,    0.0,             'h2',      0.0),
            PhaseSpec('Takeoff', TAKEOFF_DUR_MIN,TAKEOFF_POWER_KW, TAKEOFF_DIST_NM, 'h2',      0.0),
            PhaseSpec('Climb',   climb_min,      CLIMB_POWER_KW,   climb_nm,        'h2',      mid_alt),
        ],
        post_cruise_phases = [
            PhaseSpec('Descent', desc_min,       DESCENT_POWER_KW, desc_nm,         'battery', mid_alt),
            PhaseSpec('Approach',APPROACH_DUR_MIN,APPROACH_POWER_KW,APPROACH_DIST_NM,'battery', 0.0),
            PhaseSpec('Landing', 0.0,            0.0,              LANDING_DIST_NM,  'battery', 0.0),
        ],
    )


# ── Sweeps ────────────────────────────────────────────────────────────────────

def fc_sizing_sweep(cruise_alt_ft: float = 5000.0, n_points: int = 60):
    """Sweep FC max power at a fixed cruise altitude.

    Returns
    -------
    fc_powers [kW], ranges [nm], costs_per_nm [$/nm], togw_lbs
    """
    ac        = CargoECTOL()
    fc_powers = np.linspace(50.0, 350.0, n_points)
    ranges       = np.zeros(n_points)
    costs_per_nm = np.full(n_points, np.nan)
    togw_lbs_arr = np.zeros(n_points)
    mission      = _build_mission(cruise_alt_ft)

    for k, fc_kw in enumerate(fc_powers):
        h2_cfg  = _build_h2_config(fc_kw)
        prop    = _build_propulsion(fc_kw)
        fuel_kg = _total_h2_fuel_kg(fc_kw, h2_cfg)

        result = solve_mission(
            aircraft        = ac,
            propulsion      = prop,
            mission         = mission,
            h2_fuel_mass_kg = fuel_kg,
            payload_kg      = PAYLOAD_KG,
        )
        ranges[k]       = result.total_range_nm
        togw_lbs_arr[k] = result.start_weight_lbs
        if result.total_range_nm > 0:
            costs_per_nm[k] = result.cost()['total_cost_usd'] / result.total_range_nm

    return fc_powers, ranges, costs_per_nm, togw_lbs_arr


def altitude_contour_sweep(
    fc_powers: np.ndarray = None,
    altitudes: np.ndarray = None,
):
    """Sweep FC max power × cruise altitude and record range at each point.

    Returns
    -------
    fc_powers, altitudes, range_grid, mtow_ok_grid
    range_grid shape: (len(altitudes), len(fc_powers))
    """
    if fc_powers is None:
        fc_powers = np.linspace(50.0, 300.0, 30)
    if altitudes is None:
        altitudes = np.linspace(2000.0, 12000.0, 20)

    ac           = CargoECTOL()
    range_grid   = np.zeros((len(altitudes), len(fc_powers)))
    mtow_ok_grid = np.zeros_like(range_grid, dtype=bool)

    for i, alt_ft in enumerate(altitudes):
        mission = _build_mission(alt_ft)
        for j, fc_kw in enumerate(fc_powers):
            h2_cfg  = _build_h2_config(fc_kw)
            prop    = _build_propulsion(fc_kw)
            fuel_kg = _total_h2_fuel_kg(fc_kw, h2_cfg)

            result = solve_mission(
                aircraft        = ac,
                propulsion      = prop,
                mission         = mission,
                h2_fuel_mass_kg = fuel_kg,
                payload_kg      = PAYLOAD_KG,
            )
            range_grid[i, j]   = result.total_range_nm
            mtow_ok_grid[i, j] = result.start_weight_lbs <= ac.mtow_lbs

    return fc_powers, altitudes, range_grid, mtow_ok_grid


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_fc_sweep(cruise_alt_ft: float = 5000.0, n_points: int = 60):
    """Dual-axis plot: range and cost/nm vs. FC max power."""
    fc_powers, ranges, costs_per_nm, togw_lbs = \
        fc_sizing_sweep(cruise_alt_ft, n_points)

    ac = CargoECTOL()

    mtow_mask  = togw_lbs > ac.mtow_lbs
    mtow_limit = fc_powers[np.argmax(mtow_mask)] if mtow_mask.any() else None

    feasible_mask = ~mtow_mask
    if feasible_mask.any():
        opt_idx = int(np.argmax(np.where(feasible_mask, ranges, -np.inf)))
        opt_fc  = fc_powers[opt_idx]
        opt_rng = ranges[opt_idx]
    else:
        opt_fc = opt_rng = None

    C_RNG  = (0.15, 0.40, 0.70)
    C_COST = (0.85, 0.45, 0.10)

    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax2 = ax1.twinx()

    ax1.plot(fc_powers, ranges, color=C_RNG, linewidth=1.8, label='Range')
    ax1.fill_between(fc_powers, ranges, alpha=0.10, color=C_RNG)
    ax2.plot(fc_powers, costs_per_nm, color=C_COST, linewidth=1.8,
             linestyle='--', label='Cost / nm')

    if mtow_limit is not None:
        ax1.axvline(mtow_limit, color='red', linewidth=1.2, linestyle=':',
                    label=f'MTOW limit  ({mtow_limit:.0f} kW)')
        ax1.axvspan(mtow_limit, fc_powers[-1], alpha=0.06, color='red')

    if opt_fc is not None:
        print(f"\nFC Sizing Optimum  (cruise {cruise_alt_ft:.0f} ft):")
        print(f"  FC max power  : {opt_fc:.0f} kW")
        print(f"  Range         : {opt_rng:.1f} nm")
        print(f"  Cost / nm     : ${costs_per_nm[opt_idx]:.2f} /nm")
        ax1.axvline(opt_fc, color=C_RNG, linewidth=1.0, linestyle='--', alpha=0.6)
        ax1.plot(opt_fc, opt_rng, 'o', color=C_RNG, markersize=7,
                 label=f'Optimum  {opt_fc:.0f} kW → {opt_rng:.1f} nm')

    ax1.set_xlabel('FC max power (kW)')
    ax1.set_ylabel('Total mission range (nm)', color=C_RNG)
    ax1.tick_params(axis='y', labelcolor=C_RNG)
    ax2.set_ylabel('Mission cost per nm ($/nm)', color=C_COST)
    ax2.tick_params(axis='y', labelcolor=C_COST)
    ax1.set_ylim(bottom=0)
    ax2.set_ylim(bottom=0)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, fontsize=9, loc='upper left')
    ax1.set_title(
        f'FC Sizing Study (4-bay fuel load) — Cargo eCTOL  |  350 bar H2  |  '
        f'Cruise {cruise_alt_ft:.0f} ft  |  Payload {PAYLOAD_KG:.0f} kg',
        fontsize=10,
    )
    fig.tight_layout()
    return fig


def plot_altitude_contour(fc_powers: np.ndarray = None,
                          altitudes: np.ndarray = None):
    """Contour map of range over FC max power × cruise altitude."""
    fc_powers, altitudes, range_grid, mtow_ok_grid = \
        altitude_contour_sweep(fc_powers, altitudes)

    # Global optimum within MTOW-feasible region
    feasible = mtow_ok_grid & (range_grid > 0)
    if feasible.any():
        idx     = np.unravel_index(
                      int(np.argmax(np.where(feasible, range_grid, -np.inf))),
                      range_grid.shape)
        opt_fc  = fc_powers[idx[1]]
        opt_alt = altitudes[idx[0]]
        opt_rng = range_grid[idx]
    else:
        opt_fc = opt_alt = opt_rng = None

    fig, ax = plt.subplots(figsize=(10, 6))

    cf = ax.contourf(fc_powers, altitudes / 1000.0, range_grid,
                     levels=20, cmap='viridis')
    cbar = fig.colorbar(cf, ax=ax, pad=0.02)
    cbar.set_label('Total mission range (nm)', fontsize=10)

    cs = ax.contour(fc_powers, altitudes / 1000.0, range_grid,
                    levels=10, colors='white', linewidths=0.6, alpha=0.5)
    ax.clabel(cs, fmt='%.0f nm', fontsize=7, inline=True)

    # MTOW boundary
    ax.contour(fc_powers, altitudes / 1000.0, mtow_ok_grid.astype(float),
               levels=[0.5], colors='red', linewidths=1.8, linestyles='--')
    ax.plot([], [], color='red', linewidth=1.8, linestyle='--', label='MTOW limit')

    if opt_fc is not None:
        print("\nGlobal Optimum  (FC + altitude sweep):")
        print(f"  FC max power  : {opt_fc:.0f} kW")
        print(f"  Cruise alt    : {opt_alt:.0f} ft")
        print(f"  Range         : {opt_rng:.1f} nm")
        ax.plot(opt_fc, opt_alt / 1000.0, '*', color='white', markersize=14,
                markeredgecolor='black', markeredgewidth=0.8,
                label=f'Optimum  {opt_fc:.0f} kW, {opt_alt:.0f} ft → {opt_rng:.1f} nm')

    ax.set_xlabel('FC max power (kW)')
    ax.set_ylabel('Cruise altitude (×1000 ft)')
    ax.legend(fontsize=9, loc='upper right')
    ax.set_title(
        f'Range Contour (4-bay fuel load) — Cargo eCTOL  |  350 bar H2  |  Payload {PAYLOAD_KG:.0f} kg',
        fontsize=10,
    )
    fig.tight_layout()
    return fig


# ── Optimal-FC single mission run ────────────────────────────────────────────

def run_optimal_mission(cruise_alt_ft: float = 5000.0) -> None:
    """Find the range-maximising FC size at cruise_alt_ft, then run the full
    CHAPIE solver with that design and print detailed mission outputs.

    This demonstrates how to take optimization results and feed them directly
    into the main solver.
    """
    print(f'\nFinding optimal FC at {cruise_alt_ft:.0f} ft cruise...')
    fc_powers, ranges, costs_per_nm, togw_lbs = fc_sizing_sweep(cruise_alt_ft)

    ac = CargoECTOL()
    mtow_mask     = togw_lbs > ac.mtow_lbs
    feasible_mask = ~mtow_mask
    if not feasible_mask.any():
        print('  No feasible design within MTOW — cannot run single mission.')
        return

    opt_idx = int(np.argmax(np.where(feasible_mask, ranges, -np.inf)))
    opt_fc  = fc_powers[opt_idx]

    print(f'  Optimal FC max power : {opt_fc:.1f} kW')
    print(f'  Predicted range      : {ranges[opt_idx]:.1f} nm')
    print(f'\nRunning full CHAPIE mission with {opt_fc:.1f} kW FC...')

    h2_cfg   = _build_h2_config(opt_fc)
    prop     = _build_propulsion(opt_fc)
    mission  = _build_mission(cruise_alt_ft)
    fuel_kg  = _total_h2_fuel_kg(opt_fc, h2_cfg)

    result = solve_mission(
        aircraft        = ac,
        propulsion      = prop,
        mission         = mission,
        h2_fuel_mass_kg = fuel_kg,
        payload_kg      = PAYLOAD_KG,
    )

    mtow_ok = result.start_weight_lbs <= ac.mtow_lbs
    costs   = result.cost()

    fc_mass_kg = opt_fc / FC_SPECIFIC_POWER_KW_PER_KG
    print(f"\nCargo eCTOL  |  350 bar H2  |  Optimal FC  |  Payload: {PAYLOAD_KG:.1f} kg  "
          f"({PAYLOAD_KG * KG_TO_LBS:.0f} lbs)")
    print(f"  FC max power     : {opt_fc:.1f} kW  ({fc_mass_kg:.1f} kg)")
    print(f"  Cruise altitude  : {cruise_alt_ft:.0f} ft")
    print(f"  H2 fuel loaded   : {fuel_kg:.2f} kg  (4 dedicated bays)")
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
    plot_soc(result, prop)
    plot_power(result, ac, mission, prop)


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running FC sizing sweep at 5000 ft cruise...')
    fig1 = plot_fc_sweep(cruise_alt_ft=5000.0)

    print('Running altitude x FC contour sweep (this may take a moment)...')
    fig2 = plot_altitude_contour()

    run_optimal_mission(cruise_alt_ft=5000.0)

    plt.show()
