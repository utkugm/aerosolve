# aero/visualizer.py
"""
Altitude-sweep performance analysis for a turbofan engine.

Sweeps from sea level to a specified ceiling, collects thrust, mass flow,
TSFC, and fuel burn at each altitude, then plots all four on a 2x2 figure.
"""
import matplotlib.pyplot as plt
import numpy as np
from CoolProp.CoolProp import PropsSI
from aerosolve.cycle.flight import FlightSimulator
from aerosolve.cycle.core import EnergyRole
from aerosolve.cycle.components import Nozzle

def _compute_performance(sim, eta_bypass_nozzle=0.97):
    """
    Extract scalar performance metrics from a solved FlightSimulator.

    Returns
    -------
    dict with keys: thrust_kN, m_dot_air, tsfc, m_dot_fuel
    """
    engine = sim.engine
    nozzle = engine.components[-1]

    # ---- Fuel flow (Calculated first as it is needed for mass addition) ----
    
    Q_in_kW = 0.0
    for i, comp in enumerate(engine.components):
        if comp.energy_role == EnergyRole.HEAT_ADDER:
            Q_in_kW += (engine.states[i + 1].h - engine.states[i].h) * engine.m_dot_core / 1000.0

    # LHV is fetched dynamically from the engine object
    m_dot_fuel = (Q_in_kW * 1000.0) / engine.fuel_lhv   

    # ---- Core thrust (Momentum + Pressure thrust and fuel mass added) ----
    m_dot_core_exit = engine.m_dot_core + m_dot_fuel
    V_exit_core = nozzle.V_exit
    core_exit_state = engine.states[-1]

    # Nozzle exit area and pressure thrust calculations
    rho_exit = core_exit_state.rho
    if rho_exit is None or rho_exit <= 0:
        A_exit = 0.0
    else:
        A_exit = m_dot_core_exit / (rho_exit * V_exit_core) if V_exit_core > 0 else 0.0

    thrust_core_mom = (m_dot_core_exit * V_exit_core) - (engine.m_dot_core * sim.V_flight)
    thrust_core_press = A_exit * (core_exit_state.P - engine.P_ambient)
    
    thrust_core = thrust_core_mom + thrust_core_press

    # ---- Bypass thrust (Realistic nozzle with Choked Flow logic) ----
    thrust_bypass = 0.0
    if engine.bpr > 0 and engine.bypass_exit_state is not None:

        bp_nozzle = Nozzle(P_ambient=engine.P_ambient, eta_is=eta_bypass_nozzle, name="Bypass Nozzle")
        bp_exit_state = bp_nozzle.process(engine.bypass_exit_state)
        
        V_exit_bypass = bp_nozzle.V_exit
        rho_exit_bp = bp_exit_state.rho
        
        if rho_exit_bp is None or rho_exit_bp <= 0:
            A_exit_bp = 0.0
        else:
            A_exit_bp = engine.m_dot_bypass / (rho_exit_bp * V_exit_bypass) if V_exit_bypass > 0 else 0.0
            
        thrust_bp_mom = engine.m_dot_bypass * (V_exit_bypass - sim.V_flight)
        thrust_bp_press = A_exit_bp * (bp_exit_state.P - engine.P_ambient)
        
        thrust_bypass = thrust_bp_mom + thrust_bp_press

    thrust_kN = (thrust_core + thrust_bypass) / 1000.0

    tsfc       = (m_dot_fuel * 3600.0) / thrust_kN if thrust_kN > 0 else 0.0

    return {
        "thrust_kN":  thrust_kN,
        "m_dot_air":  sim.m_dot_air,
        "tsfc":       tsfc,
        "m_dot_fuel": m_dot_fuel,
    }


def run_altitude_sweep(inlet_area=2.5, mach=0.8, bpr=8.0,
                       alt_max_m=15000, n_points=31):
    """
    Sweep from sea level to alt_max_m and plot four performance parameters.

    Parameters
    ----------
    inlet_area  : inlet capture area [m²]
    mach        : cruise Mach number [-]
    bpr         : bypass ratio [-]
    alt_max_m   : ceiling altitude [m]
    n_points    : number of equally-spaced altitude steps
    """
    altitudes  = np.linspace(0, alt_max_m, n_points)
    thrusts    = []
    mass_flows = []
    tsfcs      = []
    fuel_burns = []

    print(f"Altitude sweep — inlet_area={inlet_area} m², Mach={mach}, BPR={bpr}")

    for alt in altitudes:
        sim = FlightSimulator(altitude_m=alt, mach=mach, inlet_area=inlet_area)
        sim.build_turbofan(bpr=bpr, T_max=1400.0)
        sim.engine.run()

        perf = _compute_performance(sim)
        thrusts.append(perf["thrust_kN"])
        mass_flows.append(perf["m_dot_air"])
        tsfcs.append(perf["tsfc"])
        fuel_burns.append(perf["m_dot_fuel"])

    # ---- Plotting ----
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(
        f"Engine Performance vs Altitude\n"
        f"(A_inlet={inlet_area} m²,  Mach={mach},  BPR={bpr})",
        fontsize=14
    )

    axs[0, 0].plot(altitudes / 1000, thrusts, 'b-', linewidth=2)
    axs[0, 0].set_title("Net Thrust vs Altitude")
    axs[0, 0].set_ylabel("Thrust [kN]")
    axs[0, 0].set_xlabel("Altitude [km]")
    axs[0, 0].grid(True)

    axs[0, 1].plot(altitudes / 1000, mass_flows, 'g-', linewidth=2)
    axs[0, 1].set_title("Inlet Mass Flow vs Altitude")
    axs[0, 1].set_ylabel("m_dot_air [kg/s]")
    axs[0, 1].set_xlabel("Altitude [km]")
    axs[0, 1].grid(True)

    axs[1, 0].plot(altitudes / 1000, tsfcs, 'r-', linewidth=2)
    axs[1, 0].set_title("TSFC vs Altitude")
    axs[1, 0].set_ylabel("TSFC [kg/(kN·h)]")
    axs[1, 0].set_xlabel("Altitude [km]")
    axs[1, 0].grid(True)

    axs[1, 1].plot(altitudes / 1000, fuel_burns, 'm-', linewidth=2)
    axs[1, 1].set_title("Fuel Flow vs Altitude")
    axs[1, 1].set_ylabel("m_dot_fuel [kg/s]")
    axs[1, 1].set_xlabel("Altitude [km]")
    axs[1, 1].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_altitude_sweep()
