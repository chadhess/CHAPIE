"""
atmosphere.py — ISA atmosphere model and airspeed conversions.

Valid in the troposphere (below ~36,000 ft / 11,000 m).
"""
import math

_LAPSE_K_PER_FT = 6.87559e-6   # ISA tropospheric temperature lapse rate [K/ft]


def temperature_ratio(alt_ft: float) -> float:
    """ISA temperature ratio θ = T/T_SL."""
    return 1.0 - _LAPSE_K_PER_FT * alt_ft


def density_ratio(alt_ft: float) -> float:
    """ISA density ratio σ = ρ/ρ_SL."""
    return temperature_ratio(alt_ft) ** 4.2561


def cas_to_tas(cas_kts: float, alt_ft: float) -> float:
    """
    Convert calibrated airspeed to true airspeed [kts].
    Uses the incompressible approximation — valid below ~250 kts and 10,000 ft.
    """
    return cas_kts / math.sqrt(density_ratio(alt_ft))


def tas_to_cas(tas_kts: float, alt_ft: float) -> float:
    """Convert true airspeed to calibrated airspeed [kts]."""
    return tas_kts * math.sqrt(density_ratio(alt_ft))
