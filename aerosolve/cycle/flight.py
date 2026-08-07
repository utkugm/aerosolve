# aero/flight.py
from .environment import get_ambient_conditions, mach_to_velocity
from .engine import AeroEngine
from .components import Diffuser, Compressor, Combustor, Turbine, Nozzle


class FlightSimulator:
    """
    High-level user interface for the aero-engine library.

    Sets up atmospheric conditions and inlet mass flow from the given flight
    point, then provides factory methods to assemble engine flowpaths.

    Parameters
    ----------
    altitude_m  : pressure altitude [m]
    mach        : flight Mach number [-]
    inlet_area  : effective inlet capture area [m²]
                  Mass flow is derived as: m_dot = rho * A * max(V, 50 m/s)
                  The 50 m/s floor prevents m_dot collapsing to zero at V=0
                  (static ground run / sea-level static test).
    """

    def __init__(self, altitude_m=0, mach=0.0, inlet_area=1.5):
        self.altitude_m = altitude_m
        self.mach       = mach
        self.inlet_area = inlet_area

        # --- Atmosphere (ISA model) ---
        self.P_amb, self.T_amb, self.rho_amb = get_ambient_conditions(altitude_m)

        # --- Flight velocity ---
        self.V_flight = mach_to_velocity(mach, self.P_amb, self.T_amb)

        # --- Physical mass flow: m_dot = rho * A * V ---
        # A minimum effective velocity of 50 m/s is used so that a static
        # ground test still produces a realistic inlet mass flow.
        v_eff         = max(self.V_flight, 50.0)
        self.m_dot_air = self.rho_amb * self.inlet_area * v_eff

        # Engine is created (or re-created) inside build_turbojet / build_turbofan.
        self.engine = None

    # ------------------------------------------------------------------
    def build_turbojet(self, pr=15.0, T_max=1300.0, has_afterburner=False, comp_eta=0.85, comb_dp=0.03, turb_eta=0.88, nozzle_eta=0.95):
        """
        Assemble a single-spool turbojet flowpath.

        Parameters
        ----------
        pr              : overall compressor pressure ratio [-]
        T_max           : turbine inlet temperature (TIT) [K]
        has_afterburner : if True, add an afterburner combustor (T=1900 K)
                          downstream of the turbine before the nozzle
        """
        self.engine = AeroEngine(
            m_dot_air  = self.m_dot_air,
            V_flight   = self.V_flight,
            T_ambient  = self.T_amb,
            P_ambient  = self.P_amb,
            bypass_ratio = 0.0
        )

        self.engine.add_component(Diffuser(V_flight=self.V_flight))
        self.engine.add_component(Compressor(pressure_ratio=pr, eta_is=comp_eta))
        self.engine.add_component(Combustor(T_max=T_max, dp_ratio=comb_dp))
        self.engine.add_component(Turbine(eta_is=turb_eta))   # work injected by shaft matching

        if has_afterburner:
            # Afterburner reheat — typical TIT limit ~1900 K
            self.engine.add_component(
                Combustor(T_max=1900.0, dp_ratio=0.05, name="Afterburner")
            )

        self.engine.add_component(Nozzle(P_ambient=self.P_amb, eta_is=nozzle_eta))
        return self

    # ------------------------------------------------------------------
    def build_turbofan(self, fan_pr=1.6, core_pr=20.0, bpr=8.0, T_max=1500.0, fan_eta=0.85, hpc_eta=0.85, comb_dp=0.03, hpt_eta=0.90, lpt_eta=0.92, nozzle_eta=0.95, dual_spool=True):
        """
        Assemble a two-stream turbofan flowpath (single-spool, unmixed exhaust).

        The fan drives both the core and bypass streams.  The bypass stream
        exits through an ideal nozzle modelled inside AeroEngine.analyze_thrust().
        The core stream follows the standard: HPC → combustor → turbine → nozzle.

        Parameters
        ----------
        fan_pr  : fan pressure ratio [-]
        core_pr : high-pressure compressor (HPC) pressure ratio [-]
        bpr     : bypass ratio (m_dot_bypass / m_dot_core) [-]
        T_max   : combustor outlet / turbine inlet temperature [K]
        """
        self.engine = AeroEngine(
            m_dot_air    = self.m_dot_air,
            V_flight     = self.V_flight,
            T_ambient    = self.T_amb,
            P_ambient    = self.P_amb,
            bypass_ratio = bpr
        )

        self.engine.add_component(Diffuser(V_flight=self.V_flight))
        self.engine.add_component(Compressor(pressure_ratio=fan_pr, eta_is=fan_eta, name="Fan"))
        self.engine.add_component(Compressor(pressure_ratio=core_pr, eta_is=hpc_eta,
                                             name="High-Pressure Compressor"))
        self.engine.add_component(Combustor(T_max=T_max, dp_ratio=comb_dp, name="Combustor"))
        
        if dual_spool:
            self.engine.add_component(Turbine(eta_is=hpt_eta, name="High-Pressure Turbine"))
            self.engine.add_component(Turbine(eta_is=lpt_eta, name="Low-Pressure Turbine"))
        else:
            self.engine.add_component(Turbine(eta_is=hpt_eta, name="Turbine"))
            
        self.engine.add_component(Nozzle(P_ambient=self.P_amb, eta_is=nozzle_eta, name="Core Nozzle"))
        return self

    # ------------------------------------------------------------------
    def calculate(self):
        """Run the thermodynamic solution."""
        if self.engine is None:
            raise RuntimeError("No engine built. Call build_turbojet() or build_turbofan() first.")
        self.engine.run()
        return self

    # ------------------------------------------------------------------
    def show_results(self, eta_bypass_nozzle=0.97):
        """Print flight conditions and engine performance report."""
        print("\n[ FLIGHT CONDITIONS ] " + "-" * 30)
        print(f"  Altitude       : {self.altitude_m} m")
        print(f"  OAT            : {self.T_amb - 273.15:.1f} °C")
        print(f"  Static pressure: {self.P_amb / 100:.1f} mbar")
        print(f"  True airspeed  : {self.V_flight:.1f} m/s  (Mach {self.mach})")
        print(f"  Inlet mass flow: {self.m_dot_air:.1f} kg/s")
        self.engine.analyze_thrust(eta_bypass_nozzle=eta_bypass_nozzle)