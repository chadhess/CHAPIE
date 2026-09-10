"""
mission/solver.py — Euler-integration mission solver.

Power convention: all power values [kW] at the electrical DC bus.
Upstream efficiencies (FC electrochemistry, BOP, battery round-trip) are
applied internally; downstream losses (inverter, motor, gearbox) are
embedded in the aircraft's cruise_power_kW / phase power values.

Mission order: pre_cruise_phases → cruise → post_cruise_phases
"""
from ..results    import MissionResult
from ..atmosphere import cas_to_tas
from ..constants  import H2_LHV_kWh_per_kg


def solve_mission(
    aircraft,
    propulsion,
    mission,
    h2_fuel_mass_kg: float,
    payload_kg:      float,
    dt_min:          float = 1.0,
    target_range_nm: float = None,
) -> MissionResult:
    """
    Solve a full mission via Euler integration.

    Parameters
    ----------
    aircraft        : AircraftBase subclass — airframe performance model
    propulsion      : PropulsionSystem — energy source configuration
    mission         : MissionProfile — phase sequence and cruise altitude
    h2_fuel_mass_kg : H2 fuel loaded at departure [kg]; ignored if battery-only
    payload_kg      : payload mass [kg]
    dt_min          : cruise Euler time step [minutes]
    target_range_nm : total mission range target [nm].  When set, cruise
                      terminates as soon as this distance is reached rather
                      than running until energy is exhausted (max-range mode).
                      Must be greater than the sum of all fixed phase distances;
                      pass None (default) for max-range behaviour.

    Returns
    -------
    MissionResult

    H2 hybrid behaviour
    -------------------
    Pre-cruise 'h2' phases
        FC covers up to fc_max (altitude-derated for the phase's rep_alt_ft).
        Buffer battery covers any shortfall between FC output and demand.
    Cruise
        FC output is capped at its altitude-derated maximum each step.
        If FC max exceeds demand, the surplus charges the buffer battery.
        If FC max is below demand, the battery covers the shortfall and
        cruise ends when either H2 is exhausted, the battery reaches its
        reserve, or target_range_nm is reached — whichever comes first.
    Post-cruise 'battery' phases
        Battery only; FC not active.

    Battery-electric behaviour
    --------------------------
    h2_fuel_mass_kg is ignored.  All phases and cruise draw from battery.
    Cruise ends when the battery is depleted or target_range_nm is reached.
    """
    is_h2 = propulsion.h2 is not None
    dt_h  = dt_min / 60.0

    # ── Starting mass ────────────────────────────────────────────────────────
    if is_h2:
        tank_struct = propulsion.h2.tank_structure_mass_kg(h2_fuel_mass_kg)
        start_mass  = (aircraft.empty_mass_kg
                       + payload_kg
                       + propulsion.non_fuel_fixed_mass_kg
                       + tank_struct
                       + h2_fuel_mass_kg)
    else:
        start_mass = (aircraft.empty_mass_kg
                      + payload_kg
                      + propulsion.non_fuel_fixed_mass_kg)

    mass = start_mass

    # ── State ────────────────────────────────────────────────────────────────
    bat_cap = propulsion.battery.energy_kWh
    bat_e   = bat_cap           # start at 100% SOC
    h2_rem  = h2_fuel_mass_kg if is_h2 else 0.0

    h2_burned = 0.0
    bat_dis   = 0.0             # cumulative energy drawn from battery [kWh]
    bat_chg   = 0.0             # cumulative energy returned to battery [kWh]
    t         = 0.0
    range_tot = 0.0
    range_crs = 0.0
    t_crs     = 0.0
    wt        = [(0.0, mass)]   # mass_vs_time trace
    soc       = [(0.0, 1.0)]    # bat_soc_vs_time trace

    # ── Efficiencies ─────────────────────────────────────────────────────────
    eta_chg    = propulsion.battery.eta_charge
    eta_dis    = propulsion.battery.eta_discharge
    eta_fc_bop = (propulsion.h2.eta_fc * propulsion.h2.eta_BOP) if is_h2 else 1.0

    # ── Helper: run one fixed phase ──────────────────────────────────────────
    def run_phase(dur_h: float, P_elec: float, source: str, alt_ft: float = 0.0):
        nonlocal mass, bat_e, bat_dis, bat_chg, h2_rem, h2_burned, t

        P_elec       = aircraft.phase_power_kW(P_elec, mass)
        fc_max_phase = propulsion.h2.fc_max_at_alt_kw(alt_ft) if is_h2 else 0.0

        if (not is_h2) or source == 'battery':
            dE = P_elec * dur_h / eta_dis
            bat_e  -= dE
            bat_dis += dE
        else:
            P_fc  = min(fc_max_phase, P_elec)
            P_bat = max(0.0, P_elec - P_fc)
            dh2 = P_fc * dur_h / (eta_fc_bop * H2_LHV_kWh_per_kg)
            h2_rem    -= dh2
            h2_burned += dh2
            mass      -= dh2
            if P_bat > 0.0:
                dE = P_bat * dur_h / eta_dis
                bat_e  -= dE
                bat_dis += dE

        t += dur_h
        wt.append((t, mass))
        soc.append((t, bat_e / bat_cap))

    # ── Pre-cruise phases ────────────────────────────────────────────────────
    for phase in mission.pre_cruise_phases:
        dur_h = phase.dur_min / 60.0
        range_tot += phase.dist_nm
        if dur_h > 0.0:
            run_phase(dur_h, phase.power_kW, phase.source, phase.rep_alt_ft)

    # ── Cruise ───────────────────────────────────────────────────────────────
    fc_max_crs = propulsion.h2.fc_max_at_alt_kw(mission.cruise_alt_ft) if is_h2 else 0.0

    # Battery-electric: reserve energy for post-cruise phases so cruise ends
    # with exactly enough charge remaining for descent/approach/landing.
    # Mass is constant (no fuel burn), so phase powers are deterministic here.
    # H2 hybrid: battery is recharged during cruise; reserve is not needed.
    if not is_h2:
        reserve_kWh = sum(
            aircraft.phase_power_kW(p.power_kW, start_mass) * (p.dur_min / 60.0) / eta_dis
            for p in mission.post_cruise_phases if p.dur_min > 0.0
        )
    else:
        reserve_kWh = 0.0

    # Convert total-mission target to cruise-only target distance
    if target_range_nm is not None:
        target_cruise_nm = max(0.0, target_range_nm - mission.total_fixed_range_nm())
    else:
        target_cruise_nm = None

    if is_h2:
        while h2_rem > 1e-9 and bat_e > reserve_kWh + 1e-9:
            P_req = aircraft.cruise_power_kW(mass)

            # FC output is capped at its derated maximum.
            # Any shortfall below P_req is drawn from the battery.
            P_fc_max_step = min(fc_max_crs, P_req)

            # Charge battery from FC headroom if not full and FC has spare capacity
            if bat_e < bat_cap - 1e-6:
                P_fc_headroom = fc_max_crs - P_req
                if P_fc_headroom > 0.0:
                    E_deficit   = bat_cap - bat_e
                    P_chg_limit = E_deficit / (eta_chg * dt_h)
                    P_fc_chg    = min(P_fc_headroom, P_chg_limit)
                else:
                    P_fc_chg = 0.0
            else:
                P_fc_chg = 0.0

            P_fc_out    = P_fc_max_step + P_fc_chg   # capped at fc_max_crs
            P_bat_draw  = max(0.0, P_req - P_fc_max_step)  # battery covers shortfall

            speed_tas = cas_to_tas(aircraft.cruise_speed_kts(mass), mission.cruise_alt_ft)
            dist_full = speed_tas * dt_h

            if target_cruise_nm is not None and range_crs + dist_full >= target_cruise_nm:
                remaining = target_cruise_nm - range_crs
                if remaining <= 0.0:
                    break
                dt_use         = remaining / speed_tas
                dist           = remaining
                target_reached = True
            else:
                # Battery depletion partial-step (mirrors battery-electric logic)
                if P_bat_draw > 0.0:
                    available_bat = bat_e - reserve_kWh
                    dE_bat_full   = P_bat_draw * dt_h / eta_dis
                    if dE_bat_full >= available_bat:
                        dt_use         = available_bat * eta_dis / P_bat_draw
                        dist           = speed_tas * dt_use
                        target_reached = True   # reuse flag to trigger break after step
                    else:
                        dt_use         = dt_h
                        dist           = dist_full
                        target_reached = False
                else:
                    dt_use         = dt_h
                    dist           = dist_full
                    target_reached = False

            # Apply H2 consumption
            dh2_demand  = P_fc_out * dt_use / (eta_fc_bop * H2_LHV_kWh_per_kg)
            dh2         = min(dh2_demand, h2_rem)
            P_fc_actual = dh2 * eta_fc_bop * H2_LHV_kWh_per_kg / dt_use if dt_use > 0 else 0.0

            h2_rem    -= dh2
            h2_burned += dh2
            mass      -= dh2

            # Battery charging from FC headroom (only when FC exceeds demand)
            P_fc_chg_actual = max(0.0, P_fc_actual - P_fc_max_step)
            if P_fc_chg_actual > 0.0:
                dE_chg  = P_fc_chg_actual * eta_chg * dt_use
                bat_e   = min(bat_cap, bat_e + dE_chg)
                bat_chg += dE_chg

            # Battery draw for FC shortfall
            if P_bat_draw > 0.0:
                dE_bat  = P_bat_draw * dt_use / eta_dis
                bat_e  -= dE_bat
                bat_dis += dE_bat

            t         += dt_use
            t_crs     += dt_use
            range_crs += dist
            range_tot += dist
            wt.append((t, mass))
            soc.append((t, bat_e / bat_cap))

            if target_reached:
                break

    else:
        # Battery-electric: mass constant, so P_req is nearly constant
        while bat_e > reserve_kWh + 1e-9:
            P_req     = aircraft.cruise_power_kW(mass)
            speed_tas = cas_to_tas(aircraft.cruise_speed_kts(mass), mission.cruise_alt_ft)

            # Target range check takes priority over energy exhaustion
            if target_cruise_nm is not None:
                dist_full = speed_tas * dt_h
                if range_crs + dist_full >= target_cruise_nm:
                    remaining = target_cruise_nm - range_crs
                    if remaining <= 0.0:
                        break
                    dt_partial = remaining / speed_tas
                    dE         = P_req * dt_partial / eta_dis
                    bat_e     -= dE
                    bat_dis   += dE
                    t         += dt_partial
                    t_crs     += dt_partial
                    range_crs += remaining
                    range_tot += remaining
                    wt.append((t, mass))
                    soc.append((t, bat_e / bat_cap))
                    break

            dE_full   = P_req * dt_h / eta_dis
            available = bat_e - reserve_kWh

            if dE_full >= available:
                # Partial final step — use only what's available above reserve
                dt_partial = available * eta_dis / P_req
                dist       = speed_tas * dt_partial
                bat_dis   += available
                bat_e     -= available          # lands exactly at reserve_kWh
                t         += dt_partial
                t_crs     += dt_partial
                range_crs += dist
                range_tot += dist
                wt.append((t, mass))
                soc.append((t, bat_e / bat_cap))
                break

            bat_e  -= dE_full
            bat_dis += dE_full
            t       += dt_h
            t_crs   += dt_h
            dist     = speed_tas * dt_h
            range_crs += dist
            range_tot += dist
            wt.append((t, mass))
            soc.append((t, bat_e / bat_cap))

    # ── Post-cruise phases ───────────────────────────────────────────────────
    for phase in mission.post_cruise_phases:
        dur_h = phase.dur_min / 60.0
        range_tot += phase.dist_nm
        if dur_h > 0.0:
            run_phase(dur_h, phase.power_kW, phase.source, phase.rep_alt_ft)

    return MissionResult(
        total_range_nm     = range_tot,
        cruise_range_nm    = range_crs,
        total_time_h       = t,
        cruise_time_h      = t_crs,
        h2_burned_kg       = h2_burned,
        bat_net_energy_kWh = bat_dis - bat_chg,
        battery_soc_final  = bat_e / bat_cap,
        start_mass_kg      = start_mass,
        end_mass_kg        = mass,
        payload_kg         = payload_kg,
        mass_vs_time       = wt,
        bat_soc_vs_time    = soc,
    )
