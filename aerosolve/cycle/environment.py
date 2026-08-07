# aero/environment.py
"""
International Standard Atmosphere (ISA) model.

Covers the troposphere (0–11 000 m, temperature lapse rate 6.5 K/km) and
the lower stratosphere (11 000–20 000 m, isothermal at 216.65 K).
"""
import math
import logging
from CoolProp.CoolProp import PropsSI

# ISA constants
_P0    = 101325.0   # Sea-level static pressure [Pa]
_T0    = 288.15     # Sea-level static temperature [K]
_L     = 0.0065     # Tropospheric lapse rate [K/m]
_R     = 287.05     # Specific gas constant for dry air [J/(kg·K)]
_g     = 9.80665    # Standard gravity [m/s²]
_R_isa = 287.05287
_GAMMA = 1.4        # Specific heat ratio for standard air


def get_ambient_conditions(altitude_m):
    """
    Return ISA static conditions at the given pressure altitude.

    Parameters
    ----------
    altitude_m : pressure altitude [m]

    Returns
    -------
    P   : static pressure [Pa]
    T   : static temperature [K]
    rho : air density [kg/m³]
    """
    if altitude_m > 20000:
        logging.warning(
            f"get_ambient_conditions: Altitude ({altitude_m} m) exceeds the 20 km limit "
            "of the ISA lower stratosphere model. The code will not crash, but physical "
            "results may be inaccurate (assumes an infinite isothermal layer)."
        )

    if altitude_m <= 11000:
        # Troposphere: linear temperature decrease
        T = _T0 - _L * altitude_m
        P = _P0 * (T / _T0) ** (_g / (_R_isa * _L))
    else:
        # Lower stratosphere: isothermal
        T_tropo = _T0 - _L * 11000.0
        P_11k   = _P0 * (T_tropo / _T0) ** (_g / (_R_isa * _L))
        T       = T_tropo
        P       = P_11k * math.exp(-_g * (altitude_m - 11000.0) / (_R_isa * T_tropo))

    rho = PropsSI('D', 'P', P, 'T', T, 'Air')
    return P, T, rho


def mach_to_velocity(mach, P_ambient, T_ambient):
    """
    Convert a Mach number to true airspeed [m/s] at the given temperature.

    Parameters
    ----------
    mach      : Mach number [-]
    T_ambient : ambient static temperature [K]

    Returns
    -------
    TAS [m/s]
    """
    # Under standard atmospheric conditions, dry air behaves very close to an ideal gas.
    # The ideal gas formula is used here for computational efficiency instead of querying CoolProp
    # repeatedly, which is beneficial for continuous iterative solutions.
    a = math.sqrt(_GAMMA * _R * T_ambient)   # Speed of sound [m/s]
    return mach * a