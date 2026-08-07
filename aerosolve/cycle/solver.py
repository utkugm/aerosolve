# aero/solver.py
"""
Numerical root-finding utilities for the aero thermodynamic cycle solver.
"""
import logging
from CoolProp.CoolProp import PropsSI


def secant_pressure_search(state_in, T_target, eta_is, mode="compressor",
                            tol=1e-4, max_iter=50):
    """
    Find the outlet pressure that yields a prescribed outlet temperature,
    using the Secant (chord) method.

    This is used when a component is specified by its outlet temperature
    rather than by a pressure ratio or work input.

    Parameters
    ----------
    state_in  : ThermodynamicState — inlet conditions
    T_target  : target outlet temperature [K]
    eta_is    : isentropic efficiency of the component [-]
    mode      : 'compressor' (pressure increases) or 'turbine' (pressure decreases)
    tol       : convergence tolerance on temperature error [K]
    max_iter  : maximum number of iterations

    Returns
    -------
    P_out : outlet pressure [Pa]

    Raises
    ------
    ValueError if the Secant iteration stalls (zero denominator).
    """
    # Initial bracket: conservatively straddle the expected answer
    if mode == "compressor":
        P1, P2 = state_in.P * 1.1, state_in.P * 2.0
    else:
        P1, P2 = state_in.P * 0.9, state_in.P * 0.5

    def temperature_error(P_guess):
        """Actual outlet temperature minus T_target for a given P_guess."""
        try:
            h_ideal = PropsSI('H', 'P', P_guess, 'S', state_in.s, state_in.fluid)
            if mode == "compressor":
                h_actual = state_in.h + (h_ideal - state_in.h) / eta_is
            else:
                h_actual = state_in.h - eta_is * (state_in.h - h_ideal)
            T_actual = PropsSI('T', 'P', P_guess, 'H', h_actual, state_in.fluid)
            return T_actual - T_target
        except Exception:
            # CoolProp cannot evaluate this pressure — steer the solver away
            # FIX 1: Return None instead of a large artificial number to indicate physical boundary exceeded.
            return None

    f1 = temperature_error(P1)
    f2 = temperature_error(P2)

    for _ in range(max_iter):
        # If CoolProp fails, apply Backtracking.
        if f2 is None:
            P2 = (P1 + P2) / 2.0
            f2 = temperature_error(P2)
            continue

        if abs(f2) < tol:
            return P2
        if f2 - f1 == 0:
            raise ValueError("secant_pressure_search: zero denominator — iteration stalled.")

        # Secant update
        P_new = P2 - f2 * (P2 - P1) / (f2 - f1)
        
        # FIX 3: Negative pressure protection. If Secant drops below zero, pull it back to a safe value.
        if P_new <= 0:
            P_new = P2 / 2.0

        P1, P2 = P2, P_new
        f1, f2 = f2, temperature_error(P2)

    logging.warning(
        f"secant_pressure_search: max iterations ({max_iter}) reached. "
        "Outlet pressure may be slightly inaccurate."
    )
    return P2