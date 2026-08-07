# aero/engine.py
import logging
from .core import ThermodynamicState, EnergyRole
from CoolProp.CoolProp import PropsSI

class AeroEngine:
    """
    Single-pass, iteration-free thermodynamic cycle manager for aero engines.

    Supports both turbojet (bypass_ratio=0) and turbofan (bypass_ratio>0) cycles.
    Shaft matching is handled automatically: compressor work is accumulated and
    injected into the next turbine as a constraint.

    Parameters
    ----------
    m_dot_air    : total inlet mass flow rate [kg/s]  (core + bypass)
    V_flight     : true airspeed [m/s]
    T_ambient    : freestream static temperature [K]
    P_ambient    : freestream static pressure [Pa]
    bypass_ratio : bypass-to-core mass flow ratio (BPR); 0 for turbojet
    """

    def __init__(self, m_dot_air, V_flight, T_ambient, P_ambient, bypass_ratio=0.0, fuel_lhv_Jkg=43000000.0):
        self.m_dot_air  = m_dot_air
        self.V_flight   = V_flight
        self.P_ambient  = P_ambient
        self.bpr        = bypass_ratio
        self.T_ambient  = T_ambient
        self.fuel_lhv   = fuel_lhv_Jkg

        # Split total flow into core and bypass streams
        self.m_dot_core   = m_dot_air / (1.0 + bypass_ratio)
        self.m_dot_bypass = m_dot_air - self.m_dot_core

        # Fan exit thermodynamic state for the bypass stream (populated during run())
        self.bypass_exit_state = None

        self.components = []

        # Seed the state list with freestream ambient conditions
        initial_state = ThermodynamicState("Air")
        initial_state.update('P', P_ambient, 'T', T_ambient)
        self.states = [initial_state]

    # ------------------------------------------------------------------
    def add_component(self, component):
        """Append a thermodynamic component to the engine flowpath."""
        self.components.append(component)

    # ------------------------------------------------------------------
    def run(self):
        """
        Execute the single-pass thermodynamic solution.

        Shaft matching:
          SHAFT_CONSUMER (compressor/fan) work is accumulated as W_shaft
          [J per kg of *core* flow].  For the fan in a turbofan, the work is
          scaled by (1 + BPR) because the fan drives the full inlet mass flow
          while the turbine only handles the core stream.
          When a SHAFT_PRODUCER (turbine) is reached, W_shaft is injected as
          its work constraint and the accumulator is reset — this pattern
          naturally extends to multi-spool designs.
        """
        engine_type = 'Turbofan' if self.bpr > 0 else 'Turbojet'
        logging.info(f"AeroEngine ({engine_type}) — single-pass solution starting...")

        unassigned_comp_work = []  # LIFO stack for compressor work demands
        fan_done = False

        # Pre-count turbines so we know when we hit the final one
        total_turbines = sum(1 for c in self.components if c.energy_role == EnergyRole.SHAFT_PRODUCER)
        turbines_processed = 0

        for comp in self.components:
            state_in = self.states[-1]

            # ---- Turbine: inject accumulated shaft work ----
            if comp.energy_role == EnergyRole.SHAFT_PRODUCER:
                turbines_processed += 1
                
                if not unassigned_comp_work:
                    logging.warning(
                        f"{comp.name}: no upstream compressor work found — "
                        "turbine work constraint set to zero."
                    )
                    comp.work = 0.0
                else:
                    # If there are more turbines downstream, pop the most recent compressor (LIFO)
                    if turbines_processed < total_turbines:
                        comp.work = unassigned_comp_work.pop()
                    # If this is the LAST turbine, it sweeps up ALL remaining work in the stack
                    else:
                        comp.work = sum(unassigned_comp_work)
                        unassigned_comp_work.clear()

            state_out = comp.process(state_in)

            # ---- Compressor / Fan: accumulate shaft work ----
            if comp.energy_role == EnergyRole.SHAFT_CONSUMER:
                delta_h = state_out.h - state_in.h  # Specific work [J/kg]

                if self.bpr > 0 and not fan_done:
                    # Fan: moves all air (core + bypass), so turbine work demand
                    # per kg of core flow is scaled by (1 + BPR).
                    unassigned_comp_work.append(delta_h * (1.0 + self.bpr))
                    self.bypass_exit_state = state_out  # Bypass stream exits here
                    fan_done  = True
                else:
                    unassigned_comp_work.append(delta_h)

            self.states.append(state_out)

        logging.info("AeroEngine — thermodynamic solution complete.")

    # ------------------------------------------------------------------
    def analyze_thrust(self, eta_bypass_nozzle= 0.97):
        """
        Compute and report net thrust, fuel flow, and TSFC.
        Includes Fuel Mass Addition and Pressure Thrust for choked nozzles.
        """
        if len(self.states) < len(self.components) + 1:
            print("ERROR: call run() before analyze_thrust().")
            return

        nozzle = self.components[-1]
        if (getattr(nozzle, 'energy_role', None) != EnergyRole.PASS_THROUGH
                or not hasattr(nozzle, 'V_exit')):
            print("ERROR: last component must be a Nozzle.")
            return

        # ---------------------------------------------------------
        # 1. FUEL MASS ADDITION
        # ---------------------------------------------------------
        Q_in_kW = 0.0
        for i, comp in enumerate(self.components):
            if comp.energy_role == EnergyRole.HEAT_ADDER:
                Q_in_kW += (self.states[i + 1].h - self.states[i].h) * self.m_dot_core / 1000.0

       
        m_dot_fuel = (Q_in_kW * 1000.0) / self.fuel_lhv  # kW -> W and dynamic LHV, Lower heating value of Jet-A1 [kJ/kg]
        
        # Core mass flow physically increases after the combustor
        m_dot_core_exit = self.m_dot_core + m_dot_fuel

        # ---------------------------------------------------------
        # 2. CORE THRUST (Momentum + Pressure)
        # ---------------------------------------------------------
        V_exit_core = nozzle.V_exit
        core_exit_state = self.states[-1]
        
        # FIX: Guard against two-phase/invalid density regions from CoolProp returning None
        rho_exit = core_exit_state.rho
        if rho_exit is None or rho_exit <= 0:
            A_exit = 0.0
        else:
            A_exit = m_dot_core_exit / (rho_exit * V_exit_core) if V_exit_core > 0 else 0.0
        
        thrust_core_mom = (m_dot_core_exit * V_exit_core) - (self.m_dot_core * self.V_flight)
        
        # Pressure Thrust (only positive if nozzle is choked and P_exit > P_ambient)
        thrust_core_press = A_exit * (core_exit_state.P - self.P_ambient)
        
        thrust_core = thrust_core_mom + thrust_core_press

        # ---------------------------------------------------------
        # 3. BYPASS THRUST (Realistic Nozzle with Choked Flow logic)
        # ---------------------------------------------------------
        thrust_bypass = 0.0
        V_exit_bypass = 0.0
        
        if self.bpr > 0 and self.bypass_exit_state is not None:
            # FIX: Instantiate a Nozzle for the bypass stream to apply the choked-flow 
            # bisection search and capture pressure thrust if the bypass stream is choked.
            from .components import Nozzle
            bp_nozzle = Nozzle(P_ambient=self.P_ambient, eta_is=eta_bypass_nozzle, name="Bypass Nozzle")
            bp_exit_state = bp_nozzle.process(self.bypass_exit_state)
            
            V_exit_bypass = bp_nozzle.V_exit
            rho_exit_bp = bp_exit_state.rho
            
            # Guard against two-phase/invalid density regions
            if rho_exit_bp is None or rho_exit_bp <= 0:
                A_exit_bp = 0.0
            else:
                A_exit_bp = self.m_dot_bypass / (rho_exit_bp * V_exit_bypass) if V_exit_bypass > 0 else 0.0
            
            thrust_bp_mom = (self.m_dot_bypass * V_exit_bypass) - (self.m_dot_bypass * self.V_flight)
            thrust_bp_press = A_exit_bp * (bp_exit_state.P - self.P_ambient)
            
            thrust_bypass = thrust_bp_mom + thrust_bp_press
            
            # Save bypass nozzle state for the formatted report
            self._bp_is_choked = bp_nozzle.is_choked
            self._bp_P_exit = bp_exit_state.P
            self._bp_press_thrust = thrust_bp_press

        thrust_kN = (thrust_core + thrust_bypass) / 1000.0

        # TSFC in SI aviation units: kg fuel per kN per hour
        TSFC = (m_dot_fuel * 3600.0) / thrust_kN if thrust_kN > 0 else 0.0

        # ---------------------------------------------------------
        # 4. FORMATTED REPORT
        # ---------------------------------------------------------
        # Calculate local speed of sound dynamically
        local_a = PropsSI('A', 'P', self.P_ambient, 'T', self.T_ambient, 'Air')
        mach_actual = self.V_flight / local_a
        
        print("\n" + "=" * 52)
        print("  THRUST REPORT")
        print("=" * 52)
        print(f"  Flight speed  (M~{mach_actual:.2f})     : {self.V_flight:>7.1f} m/s")
        print(f"  Total inlet mass flow      : {self.m_dot_air:>7.1f} kg/s")
        print(f"  Core exit mass flow        : {m_dot_core_exit:>7.2f} kg/s (Fuel Added)")
        print(f"  Core nozzle exit velocity  : {V_exit_core:>7.1f} m/s")
        
        # Check if the choked logic from components.py flagged this nozzle
        if getattr(nozzle, 'is_choked', False):
            print("  Core nozzle status         :   CHOKED (P_exit > P_amb)")
            print(f"  Nozzle exit pressure       : {core_exit_state.P/1e5:>7.3f} bar")
            print(f"  Pressure Thrust Contrib.   :{thrust_core_press/1000.0:>7.2f} kN")
        else:
            print("  Core nozzle status         :   UNCHOKED (Perfect Expansion)")
            print(f"  Nozzle exit pressure       :  {core_exit_state.P/1e5:>7.3f} bar (Fully Expanded)")
             
        if self.bpr > 0:
            print(f"  Bypass nozzle exit vel.    : {V_exit_bypass:>7.1f} m/s")
            # Print accurate choked status for bypass stream
            if getattr(self, '_bp_is_choked', False):
                print("  Bypass nozzle status       :   CHOKED (P_exit > P_amb)")
                print(f"  Bypass exit pressure       : {self._bp_P_exit/1e5:>7.3f} bar")
                print(f"  Bypass Press. Thrust       :{self._bp_press_thrust/1000.0:>7.2f} kN")
            else:
                print("  Bypass nozzle status       :   UNCHOKED (Perfect Expansion)")
        print("-" * 52)

        if thrust_kN > 0:
            print(f"  Net thrust                 : {thrust_kN:>7.2f} kN")
            print(f"  Fuel flow (m_dot_f)        : {m_dot_fuel:>7.3f} kg/s")
            print(f"  TSFC                       :  {TSFC:>7.3f} kg/(kN*h)")
        else:
            print("  NO NET THRUST — exit velocity <= flight velocity.")
            print(f"  Fuel consumed: {m_dot_fuel:.3f} kg/s with no net propulsive force.")

        print("=" * 52)
