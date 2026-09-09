"""
plotting.py — Visualization utilities for CHAPIE mission results.

Requires matplotlib (not a core CHAPIE dependency — import explicitly):
    from CHAPIE.plotting import plot_mission

Each plot function accepts an optional ax argument so panels can be embedded
into any figure layout.  plot_mission() is a convenience wrapper that
produces a self-contained 3-panel figure (weight, SOC, power vs time).
"""


# ── Internal helpers ──────────────────────────────────────────────────────────

def _phase_boundaries(mission, result):
    """
    Return [(t_min, label), ...] for the start of every phase, including cruise.
    Used to draw vertical boundary markers across all panels.
    """
    wt = result.mass_vs_time

    n_pre_ran  = sum(1 for p in mission.pre_cruise_phases  if p.dur_min > 0)
    n_post_ran = sum(1 for p in mission.post_cruise_phases if p.dur_min > 0)

    boundaries = []
    t_h = 0.0

    for phase in mission.pre_cruise_phases:
        boundaries.append((t_h * 60, phase.name))
        if phase.dur_min > 0:
            t_h += phase.dur_min / 60.0

    # Cruise: use the actual timestamp from mass_vs_time
    crs_start_t_min = wt[n_pre_ran][0] * 60
    boundaries.append((crs_start_t_min, 'Cruise'))

    # Post-cruise: start from cruise-end timestamp
    t_h = wt[len(wt) - n_post_ran - 1][0]
    for phase in mission.post_cruise_phases:
        if phase.dur_min == 0.0:
            continue
        boundaries.append((t_h * 60, phase.name))
        t_h += phase.dur_min / 60.0

    return boundaries


def _power_series(result, aircraft, mission, propulsion):
    """
    Reconstruct power time series at true simulation resolution.

    Fixed phases: step pairs (horizontal bar with vertical jumps at boundaries).
    Cruise: one point per Euler step, giving the mass-varying power trace.

    Returns
    -------
    t_min   : list of times [min]
    P_fc    : list of FC power values [kW]
    P_bat   : list of battery power values [kW]
    """
    is_h2 = result.h2_burned_kg > 0.0
    wt    = result.mass_vs_time

    n_pre  = 1 + sum(1 for p in mission.pre_cruise_phases  if p.dur_min > 0)
    n_post =     sum(1 for p in mission.post_cruise_phases if p.dur_min > 0)
    i_crs_start = n_pre
    i_crs_end   = len(wt) - n_post

    def _split(P, source, alt_ft=0.0):
        if is_h2 and source == 'h2' and propulsion.h2 is not None:
            pfc = min(propulsion.h2.fc_max_at_alt_kw(alt_ft), P)
            return pfc, max(0.0, P - pfc)
        return 0.0, P

    t_out, Pfc_out, Pbat_out = [], [], []

    # Pre-cruise fixed phases
    t_h    = 0.0
    wt_idx = 0
    for phase in mission.pre_cruise_phases:
        if phase.dur_min == 0.0:
            continue
        P = aircraft.phase_power_kW(phase.power_kW, wt[wt_idx][1])
        dur_h = phase.dur_min / 60.0
        pfc, pbat = _split(P, phase.source, phase.rep_alt_ft)
        t_out    += [t_h * 60, (t_h + dur_h) * 60]
        Pfc_out  += [pfc,  pfc]
        Pbat_out += [pbat, pbat]
        t_h    += dur_h
        wt_idx += 1

    # Cruise — bridge point ensures vertical drop at transition
    bridge_mass = wt[i_crs_start - 1][1]
    bridge_t    = wt[i_crs_start - 1][0]
    cruise_seq  = [(bridge_t, bridge_mass)] + [wt[i] for i in range(i_crs_start, i_crs_end)]
    fc_max_crs  = propulsion.h2.fc_max_at_alt_kw(mission.cruise_alt_ft) if (is_h2 and propulsion.h2) else 0.0

    for t_step_h, m_step in cruise_seq:
        P_req = aircraft.cruise_power_kW(m_step)
        pfc   = min(fc_max_crs, P_req) if is_h2 else 0.0
        pbat  = 0.0 if is_h2 else P_req
        t_out.append(t_step_h * 60)
        Pfc_out.append(pfc)
        Pbat_out.append(pbat)

    # Post-cruise fixed phases
    t_h    = wt[i_crs_end - 1][0]
    wt_idx = i_crs_end - 1
    for phase in mission.post_cruise_phases:
        if phase.dur_min == 0.0:
            continue
        P = aircraft.phase_power_kW(phase.power_kW, wt[wt_idx][1])
        dur_h = phase.dur_min / 60.0
        pfc, pbat = _split(P, phase.source, phase.rep_alt_ft)
        t_out    += [t_h * 60, (t_h + dur_h) * 60]
        Pfc_out  += [pfc,  pfc]
        Pbat_out += [pbat, pbat]
        t_h    += dur_h
        wt_idx += 1

    return t_out, Pfc_out, Pbat_out


def _draw_phase_boundaries(ax, boundaries, label_phases=True):
    """Draw vertical dashed lines and optional phase labels on an axes."""
    for t_min, name in boundaries:
        ax.axvline(t_min, color='gray', linewidth=0.7, linestyle='--', alpha=0.6)
    if label_phases:
        y_lim = ax.get_ylim()
        y_label = y_lim[1] * 0.97
        for t_min, name in boundaries:
            # Place label at midpoint between this and next boundary
            next_t = boundaries[boundaries.index((t_min, name)) + 1][0] \
                     if boundaries.index((t_min, name)) < len(boundaries) - 1 \
                     else ax.get_xlim()[1]
            mid_t = (t_min + next_t) / 2
            ax.text(mid_t, y_label, name, ha='center', va='top',
                    fontsize=7.5, color='dimgray')


# ── Public plot functions ─────────────────────────────────────────────────────

def plot_weight(result, ax=None, title=None, label_phases=True,
                mission=None, color='steelblue'):
    """
    Plot aircraft gross weight vs mission time.

    Parameters
    ----------
    result        : MissionResult
    ax            : matplotlib Axes; created if None
    title         : axes title string
    label_phases  : draw phase boundary lines (requires mission)
    mission       : MissionProfile; required when label_phases=True
    color         : line color
    """
    import matplotlib.pyplot as plt
    from .constants import KG_TO_LBS

    if ax is None:
        _, ax = plt.subplots(figsize=(11, 4))

    t_min = [pt[0] * 60 for pt in result.mass_vs_time]
    w_lbs = [pt[1] * KG_TO_LBS for pt in result.mass_vs_time]

    ax.plot(t_min, w_lbs, color=color, linewidth=1.5)
    ax.set_ylabel('Gross weight [lbs]')
    ax.set_xlabel('Mission time [min]')
    if title:
        ax.set_title(title, fontsize=10)
    ax.grid(True, linewidth=0.4, alpha=0.5)

    if label_phases and mission is not None:
        boundaries = _phase_boundaries(mission, result)
        _draw_phase_boundaries(ax, boundaries)

    return ax


def plot_soc(result, propulsion, ax=None, title=None, label_phases=True,
             mission=None, color='darkorange'):
    """
    Plot battery state of charge vs mission time.

    Parameters
    ----------
    result        : MissionResult
    propulsion    : PropulsionSystem (used for battery capacity label)
    ax            : matplotlib Axes; created if None
    title         : axes title string
    label_phases  : draw phase boundary lines (requires mission)
    mission       : MissionProfile; required when label_phases=True
    color         : line color
    """
    import matplotlib.pyplot as plt

    if ax is None:
        _, ax = plt.subplots(figsize=(11, 5))

    t_min = [pt[0] * 60 for pt in result.bat_soc_vs_time]
    soc_pct = [pt[1] * 100 for pt in result.bat_soc_vs_time]

    ax.plot(t_min, soc_pct, color=color, linewidth=1.5)
    ax.set_ylim(-5, 110)
    ax.axhline(100, color='gray', linewidth=0.6, linestyle=':')
    ax.axhline(0,   color='gray', linewidth=0.6, linestyle=':')
    ax.set_ylabel('Battery SOC [%]')
    ax.set_xlabel('Mission time [min]')
    cap = propulsion.battery.energy_kWh
    n   = propulsion.battery.n_packs
    ax.set_title(title or f'Battery SOC  ({n}-pack, {cap:.1f} kWh)', fontsize=10)
    ax.grid(True, linewidth=0.4, alpha=0.5)

    if label_phases and mission is not None:
        boundaries = _phase_boundaries(mission, result)
        _draw_phase_boundaries(ax, boundaries)

    return ax


def plot_power(result, aircraft, mission, propulsion,
               ax=None, title=None, label_phases=True):
    """
    Plot electrical power draw vs mission time, split by source.

    For H2 hybrid cases: FC power (filled) + battery contribution (stacked).
    For battery-electric: battery power only.
    A dashed reference line shows the altitude-derated FC max at cruise altitude.

    Parameters
    ----------
    result        : MissionResult
    aircraft      : AircraftBase subclass
    mission       : MissionProfile
    propulsion    : PropulsionSystem
    ax            : matplotlib Axes; created if None
    title         : axes title string
    label_phases  : draw phase boundary lines
    """
    import matplotlib.pyplot as plt

    if ax is None:
        _, ax = plt.subplots(figsize=(11, 5))

    is_h2 = result.h2_burned_kg > 0.0
    t_min, P_fc, P_bat = _power_series(result, aircraft, mission, propulsion)

    C_FC  = (0.15, 0.40, 0.70)   # blue  — FC
    C_BAT = (0.85, 0.45, 0.10)   # orange — battery

    if is_h2:
        ax.plot(t_min, P_fc, color=C_FC, linewidth=1.8,
                label='Hydrogen FC')
        ax.fill_between(t_min, 0, P_fc, color=C_FC, alpha=0.20)
        fc_ref = propulsion.h2.fc_max_at_alt_kw(mission.cruise_alt_ft)
        ax.axhline(fc_ref, color=C_FC, linewidth=0.9, linestyle=':',
                   alpha=0.7,
                   label=f'FC max @ {mission.cruise_alt_ft:.0f} ft  ({fc_ref:.0f} kW)')

    bat_lbl = ('Buffer battery'
               if is_h2 else 'Battery')
    ax.plot(t_min, P_bat, color=C_BAT, linewidth=1.8, label=bat_lbl)
    ax.fill_between(t_min, 0, P_bat, color=C_BAT, alpha=0.20)

    ax.set_ylabel('Power [kW]')
    ax.set_xlabel('Mission time [min]')
    ax.set_title(title or 'Electrical power draw', fontsize=10)
    ax.legend(fontsize=8, loc='upper right')
    ax.set_ylim(bottom=0)
    ax.grid(True, linewidth=0.4, alpha=0.5)

    if label_phases:
        boundaries = _phase_boundaries(mission, result)
        _draw_phase_boundaries(ax, boundaries, label_phases=False)

    return ax


# ── Combined figure ───────────────────────────────────────────────────────────

def plot_mission(result, aircraft, mission, propulsion,
                 title=None, figsize=(11, 15)):
    """
    Produce a 3-panel figure: weight, SOC, and power vs mission time.

    Parameters
    ----------
    result      : MissionResult
    aircraft    : AircraftBase subclass
    mission     : MissionProfile
    propulsion  : PropulsionSystem
    title       : overall figure suptitle
    figsize     : figure size (width, height) in inches

    Returns
    -------
    fig : matplotlib Figure
    """
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)

    # sharex=True hides tick labels on upper panels by default — restore them
    for ax in axes:
        ax.tick_params(labelbottom=True)

    plot_weight(result, ax=axes[0], mission=mission, label_phases=True)
    plot_soc(result, propulsion, ax=axes[1], mission=mission, label_phases=True)
    plot_power(result, aircraft, mission, propulsion, ax=axes[2], label_phases=True)

    # Only the bottom panel needs the x-axis label; upper panels keep tick numbers
    axes[0].set_xlabel('')
    axes[1].set_xlabel('')

    if title:
        fig.suptitle(title, fontsize=12)

    fig.tight_layout()
    fig.subplots_adjust(hspace=0.35)
    return fig
