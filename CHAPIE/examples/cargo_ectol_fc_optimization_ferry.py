"""
examples/cargo_ectol_fc_optimization_ferry.py — FC sizing study for the cargo eCTOL
ferry configuration (350 bar H2).

Ferry configuration: all available volume is used for H2 storage. The four
dedicated H2 bays are fully loaded, and the remaining cargo bay space (after
subtracting the FC bounding volume) is used for an additional H2 tank. Payload
is zero — cargo bay is occupied by the additional H2 tank.

Sweeps FC maximum power to find the range-maximising design within weight and
volume constraints. A second sweep over cruise altitude produces a contour map
showing the combined effect of FC sizing and cruise altitude on mission range.

Assumptions
-----------
FC specific power         : 0.75 kW/kg   (mass = P_fc / 0.75)
FC volumetric density     : 250 kW/m³    (volume = P_fc / 250;
                            reference: 150 kW base FC ≈ 0.60 m³)
Altitude derating         : proportionally scaled from base 150 kW table
Climb rate                : 714 ft/min   (derived: 5000 ft / 7 min)
Climb speed               : 120 kts      (derived: 14 nm / 7 min)
Descent rate              : 1250 ft/min  (derived: 5000 ft / 4 min)
Descent speed             : 60 kts       (derived: 4 nm / 4 min)
Takeoff and climb powers  : held constant across altitude sweep
Payload                   : 0 kg (cargo bay occupied by additional H2 tank)

Run directly:
    python cargo_ectol_fc_optimization_ferry.py
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


# ── Constants ─────────────────────────────────────────────────────────────────

FC_SPECIFIC_POWER_KW_PER_KG      = 0.75    # kW/kg — given
FC_VOLUMETRIC_DENSITY_KW_PER_M3  = 250.0   # kW/m³ — assumed

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

BAY_VOL_M3       = 1.0 * 0.6 * 0.3   # per dedicated H2 bay [m³]
N_BAYS           = 4
CARGO_BAY_RAW_M3 = 4.5 * 0.75        # usable cargo bay volume before FC subtraction [m³]

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

CLIMB_RATE_FPM    = 5000.0 / 7.0
CLIMB_SPEED_KTS   = 14.0  / (7.0  / 60.0)
DESCENT_RATE_FPM  = 5000.0 / 4.0
DESCENT_SPEED_KTS = 4.0   / (4.0  / 60.0)

PAYLOAD_KG = 0.0    # ferry — no payload


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


def _fc_volume_m3(fc_power_kw: float) -> float:
    return fc_power_kw / FC_VOLUMETRIC_DENSITY_KW_PER_M3


def _cargo_tank_space_m3(fc_power_kw: float) -> float:
    return max(0.0, CARGO_BAY_RAW_M3 - _fc_volume_m3(fc_power_kw))


def _total_h2_fuel_kg(fc_power_kw: float, h2_cfg: H2Config) -> float:
    """Ferry fuel: 4 dedicated bays + remaining cargo bay space after FC."""
    cargo_space = _cargo_tank_space_m3(fc_power_kw)
    fuel_bays   = h2_cfg.fuel_mass_kg(BAY_VOL_M3, N_BAYS)
    fuel_cargo  = h2_cfg.fuel_mass_kg(cargo_space, 1) if cargo_space > 0.0 else 0.0
    return fuel_bays + fuel_cargo


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
    """Sweep FC max power at a fixed cruise altitude (ferry fuel load).

    Returns
    -------
    fc_powers [kW], ranges [nm], costs_per_nm [$/nm], togw_lbs, vol_feasible
    """
    ac        = CargoECTOL()
    fc_powers = np.linspace(50.0, 350.0, n_points)
    ranges       = np.zeros(n_points)
    costs_per_nm = np.full(n_points, np.nan)
    togw_lbs_arr = np.zeros(n_points)
    vol_feasible = np.zeros(n_points, dtype=bool)
    mission      = _build_mission(cruise_alt_ft)

    for k, fc_kw in enumerate(fc_powers):
        h2_cfg  = _build_h2_config(fc_kw)
        prop    = _build_propulsion(fc_kw)
        fuel_kg = _total_h2_fuel_kg(fc_kw, h2_cfg)
        vol_feasible[k] = _fc_volume_m3(fc_kw) <= CARGO_BAY_RAW_M3

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

    return fc_powers, ranges, costs_per_nm, togw_lbs_arr, vol_feasible


def altitude_contour_sweep(
    fc_powers: np.ndarray = None,
    altitudes: np.ndarray = None,
):
    """Sweep FC max power × cruise altitude and record range (ferry fuel load).

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
    """Dual-axis plot: range and cost/nm vs. FC max power (ferry)."""
    fc_powers, ranges, costs_per_nm, togw_lbs, vol_feasible = \
        fc_sizing_sweep(cruise_alt_ft, n_points)

    ac = CargoECTOL()

    mtow_mask = togw_lbs > ac.mtow_lbs
    vol_mask  = ~vol_feasible

    mtow_limit = fc_powers[np.argmax(mtow_mask)] if mtow_mask.any() else None
    vol_limit  = fc_powers[np.argmax(vol_mask)]  if vol_mask.any()  else None

    feasible_mask = (~mtow_mask) & vol_feasible
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

    if vol_limit is not None:
        ax1.axvline(vol_limit, color='green', linewidth=1.2, linestyle=':',
                    label=f'Volume limit  ({vol_limit:.0f} kW)')
        ax1.axvspan(vol_limit, fc_powers[-1], alpha=0.06, color='green')

    if opt_fc is not None:
        print(f"\nFC Sizing Optimum — Ferry  (cruise {cruise_alt_ft:.0f} ft):")
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
        f'FC Sizing Study (Ferry) — Cargo eCTOL  |  350 bar H2  |  '
        f'Cruise {cruise_alt_ft:.0f} ft  |  Payload {PAYLOAD_KG:.0f} kg',
        fontsize=10,
    )
    fig.tight_layout()
    return fig


def plot_altitude_contour(fc_powers: np.ndarray = None,
                          altitudes: np.ndarray = None):
    """Contour map of range over FC max power × cruise altitude (ferry)."""
    fc_powers, altitudes, range_grid, mtow_ok_grid = \
        altitude_contour_sweep(fc_powers, altitudes)

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

    ax.contour(fc_powers, altitudes / 1000.0, mtow_ok_grid.astype(float),
               levels=[0.5], colors='red', linewidths=1.8, linestyles='--')
    ax.plot([], [], color='red', linewidth=1.8, linestyle='--', label='MTOW limit')

    if opt_fc is not None:
        print("\nGlobal Optimum — Ferry  (FC + altitude sweep):")
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
        f'Range Contour (Ferry) — Cargo eCTOL  |  350 bar H2  |  Payload {PAYLOAD_KG:.0f} kg',
        fontsize=10,
    )
    fig.tight_layout()
    return fig


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Running ferry FC sizing sweep at 5000 ft cruise...')
    fig1 = plot_fc_sweep(cruise_alt_ft=5000.0)

    print('Running ferry altitude x FC contour sweep (this may take a moment)...')
    fig2 = plot_altitude_contour()

    plt.show()
