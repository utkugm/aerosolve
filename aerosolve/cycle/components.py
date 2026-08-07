# aero/components.py
import logging
import math
from abc import ABC, abstractmethod
from CoolProp.CoolProp import PropsSI
from .core import ThermodynamicState, EnergyRole
from .solver import secant_pressure_search

# ---------------------------------------------------------------------------

class Component(ABC):
    """
    Abstract base class ensuring each component has a process method.
    This guarantees that every component transforms a state_in into a state_out.
    """

    energy_role: EnergyRole = EnergyRole.PASS_THROUGH

    def __init__(self, name="Generic Component"):
        self.name = name

    @abstractmethod
    def process(self, state_in, context=None):
        """Transform an inlet ThermodynamicState into an outlet state."""


# ---------------------------------------------------------------------------

class Diffuser(Component):
    """
    Inlet diffuser / ram intake.

    Decelerates the incoming kinetic energy of the airstream, converting it into a
    rise in total pressure (ram recovery). Isentropic efficiency eta_is
    accounts for real-gas losses in the intake duct.

    Parameters
    ----------
    V_flight : freestream velocity [m/s]
    eta_is   : isentropic efficiency [-], default 0.97
    """

    energy_role = EnergyRole.PASS_THROUGH

    def __init__(self, V_flight, eta_is=0.97, name="Inlet Diffuser"):
        super().__init__(name)
        self.V_flight = V_flight
        self.eta_is   = eta_is

    def process(self, state_in, context=None):
        if self.V_flight <= 0.0:
            return state_in   # Static ground test — no ram effect

        # Energy balance: all kinetic energy converts to stagnation enthalpy
        h_out_actual = state_in.h + 0.5 * self.V_flight ** 2

        # Isentropic ram pressure computed from the ideal enthalpy rise
        h_ideal = state_in.h + self.eta_is * 0.5 * self.V_flight ** 2
        try:
            P_out = PropsSI('P', 'H', h_ideal, 'S', state_in.s, state_in.fluid)
        except Exception as e:
            raise ValueError(f"{self.name}: CoolProp error during ram pressure calc: {e}")

        state_out = ThermodynamicState(state_in.fluid)
        state_out.update('P', P_out, 'H', h_out_actual)
        return state_out


# ---------------------------------------------------------------------------

class Compressor(Component):
    """
    Adiabatic compressor (or fan) with isentropic efficiency.

    Exactly one of the following constraints must be specified:
      pressure_ratio  — outlet-to-inlet pressure ratio (most common)
      P_out           — absolute outlet pressure [Pa]
      T_out           — target outlet temperature [K]  (solved numerically)
      work            — specific shaft work input [kJ/kg]
      h_out           — outlet specific enthalpy [kJ/kg]

    Parameters
    ----------
    eta_is : isentropic efficiency [-], default 1.0 (ideal)
    """

    energy_role = EnergyRole.SHAFT_CONSUMER

    def __init__(self, pressure_ratio=None, P_out=None, T_out=None,
                 h_out=None, work=None, eta_is=1.0, name="Compressor"):
        super().__init__(name)

        if not (0.0 < eta_is <= 1.0):
            raise ValueError(f"{self.name}: eta_is must be in (0, 1].")
        if pressure_ratio is not None and pressure_ratio <= 1.0:
            raise ValueError(f"{self.name}: pressure_ratio must be > 1.0.")

        self.rp    = pressure_ratio
        self.P_out = P_out
        self.T_out = T_out
        self.h_out = h_out * 1000.0 if h_out is not None else None  # kJ/kg → J/kg
        self.work  = work  * 1000.0 if work  is not None else None  # kJ/kg → J/kg
        self.eta_is = eta_is

    def process(self, state_in, context=None):
        if all(v is None for v in [self.P_out, self.rp, self.T_out, self.work, self.h_out]):
            raise ValueError(f"{self.name}: no operating constraint provided.")

        p_out          = None
        h_out_actual   = None
        actual_work    = None

        if self.work  is not None:
            actual_work = self.work
        elif self.h_out is not None:
            actual_work = self.h_out - state_in.h

        # Case A: outlet pressure known (direct or via ratio)
        if self.P_out is not None or self.rp is not None:
            p_out = self.P_out if self.P_out is not None else state_in.P * self.rp
            if actual_work is not None:
                h_out_actual = state_in.h + actual_work
            else:
                h_ideal      = PropsSI('H', 'P', p_out, 'S', state_in.s, state_in.fluid)
                h_out_actual = state_in.h + (h_ideal - state_in.h) / self.eta_is

        # Case B: target outlet temperature (iterative pressure search)
        elif self.T_out is not None:
            p_out        = secant_pressure_search(state_in, self.T_out, self.eta_is,
                                                  mode="compressor")
            h_ideal      = PropsSI('H', 'P', p_out, 'S', state_in.s, state_in.fluid)
            h_out_actual = state_in.h + (h_ideal - state_in.h) / self.eta_is

        # Case C: specific work / enthalpy only (pressure back-calculated)
        elif actual_work is not None:
            h_out_actual = state_in.h + actual_work
            h_ideal      = state_in.h + actual_work * self.eta_is
            try:
                p_out = PropsSI('P', 'H', h_ideal, 'S', state_in.s, state_in.fluid)
            except Exception as e:
                raise ValueError(f"{self.name}: CoolProp error: {e}")

        state_out = ThermodynamicState(state_in.fluid)
        state_out.update('P', p_out, 'H', h_out_actual)

        if state_out.s < state_in.s - 1e-4:
            raise ValueError(
                f"{self.name}: SECOND LAW VIOLATION — entropy decreased "
                f"({state_in.s:.4f} → {state_out.s:.4f} J/kg·K)."
            )
        return state_out


# ---------------------------------------------------------------------------

class Combustor(Component):
    """
    Isobaric heat-addition device (main combustor or afterburner).

    The outlet temperature is prescribed directly as T_max [K].  A fractional
    total-pressure loss dp_ratio models combustor liner and mixing losses.

    Parameters
    ----------
    T_max    : combustor outlet (turbine inlet) temperature [K]
    dp_ratio : total-pressure loss fraction [-], default 0.0
               (0.03–0.05 is typical for a modern main combustor)
    """

    energy_role = EnergyRole.HEAT_ADDER

    def __init__(self, T_max, dp_ratio=0.0, name="Combustor"):
        super().__init__(name)
        if not (0.0 <= dp_ratio < 1.0):
            raise ValueError(f"{self.name}: dp_ratio must be in [0, 1).")
        self.T_max    = T_max
        self.dp_ratio = dp_ratio

    def process(self, state_in, context=None):
        if self.T_max <= state_in.T:
            logging.warning(
                f"{self.name}: T_max ({self.T_max:.1f} K) <= inlet temperature "
                f"({state_in.T:.1f} K) — combustor is cooling the flow. "
                "Check your T_max setting."
            )
        P_out     = state_in.P * (1.0 - self.dp_ratio)
        state_out = ThermodynamicState(state_in.fluid)
        state_out.update('P', P_out, 'T', self.T_max)
        return state_out


# ---------------------------------------------------------------------------

class Turbine(Component):
    """
    Adiabatic turbine with isentropic efficiency.

    In aero-engine cycle analysis the turbine work is almost always set by the
    shaft-matching constraint (i.e. 'work' is injected by AeroEngine.run()).
    Direct expansion-ratio or temperature targets are also supported for
    stand-alone analysis.

    Parameters
    ----------
    eta_is : isentropic efficiency [-], default 1.0
    """

    energy_role = EnergyRole.SHAFT_PRODUCER

    def __init__(self, expansion_ratio=None, P_out=None, T_out=None,
                 h_out=None, work=None, eta_is=1.0, name="Turbine"):
        super().__init__(name)

        if not (0.0 < eta_is <= 1.0):
            raise ValueError(f"{self.name}: eta_is must be in (0, 1].")

        self.re    = expansion_ratio
        self.P_out = P_out
        self.T_out = T_out
        self.h_out = h_out * 1000.0 if h_out is not None else None  # kJ/kg → J/kg
        self.work  = work  * 1000.0 if work  is not None else None  # kJ/kg → J/kg
        self.eta_is = eta_is

    def process(self, state_in, context=None):
        if all(v is None for v in [self.P_out, self.re, self.T_out, self.work, self.h_out]):
            raise ValueError(f"{self.name}: no operating constraint provided.")

        p_out        = None
        h_out_actual = None
        actual_work  = None

        if self.work  is not None:
            actual_work = self.work
        elif self.h_out is not None:
            actual_work = state_in.h - self.h_out

        # Case A: outlet pressure known (direct or via expansion ratio)
        if self.P_out is not None or self.re is not None:
            p_out = self.P_out if self.P_out is not None else state_in.P / self.re
            if actual_work is not None:
                h_out_actual = state_in.h - actual_work
            else:
                h_ideal      = PropsSI('H', 'P', p_out, 'S', state_in.s, state_in.fluid)
                h_out_actual = state_in.h - self.eta_is * (state_in.h - h_ideal)

        # Case B: target outlet temperature (iterative pressure search)
        elif self.T_out is not None:
            p_out        = secant_pressure_search(state_in, self.T_out, self.eta_is,
                                                  mode="turbine")
            h_ideal      = PropsSI('H', 'P', p_out, 'S', state_in.s, state_in.fluid)
            h_out_actual = state_in.h - self.eta_is * (state_in.h - h_ideal)

        # Case C: shaft work / enthalpy known — most common in aero shaft matching
        elif actual_work is not None:
            # Absolute enthalpy check removed; rely on CoolProp limits.
            h_out_actual = state_in.h - actual_work
            h_ideal      = state_in.h - actual_work / self.eta_is
            try:
                p_out = PropsSI('P', 'H', h_ideal, 'S', state_in.s, state_in.fluid)
            except Exception as e:
                raise ValueError(f"{self.name}: The requested work ({actual_work/1000:.1f} kJ/kg) might exceed the fluid's capacity. Engine stall. Error: {e}")

        state_out = ThermodynamicState(state_in.fluid)
        state_out.update('P', p_out, 'H', h_out_actual)

        if state_out.s < state_in.s - 1e-4:
            raise ValueError(
                f"{self.name}: SECOND LAW VIOLATION — entropy decreased "
                f"({state_in.s:.4f} → {state_out.s:.4f} J/kg·K)."
            )
        return state_out


# ---------------------------------------------------------------------------

class Nozzle(Component):
    """
    Convergent (or convergent-divergent) propulsive nozzle.

    Expands the working fluid from the turbine exit pressure to ambient,
    converting remaining enthalpy into exit kinetic energy.  The computed
    exit velocity V_exit [m/s] is stored as an instance attribute for use
    in the thrust calculation.

    Parameters
    ----------
    P_ambient : ambient static pressure [Pa]
    eta_is    : isentropic efficiency [-], default 0.95
    """

    energy_role = EnergyRole.PASS_THROUGH

    def __init__(self, P_ambient, eta_is=0.95, name="Nozzle"):
        super().__init__(name)
        self.P_ambient = P_ambient
        self.eta_is    = eta_is
        self.V_exit    = 0.0   # Populated after process() is called

    def process(self, state_in, context=None):
        if state_in.P <= self.P_ambient:
            # FIX: Hata fırlatıp kodu çökertmek yerine sadece uyarı veriyoruz
            logging.warning(
                f"{self.name}: INFEASIBLE CYCLE. Inlet pressure ({state_in.P/1e5:.3f} bar) "
                f"<= ambient ({self.P_ambient/1e5:.3f} bar). The cycle does not close "
                f"(insufficient enthalpy budget from upstream components)."
            )
            self.V_exit = 0.0
            self.is_choked = False
            
            # Sonraki hesaplamaların (engine.py içindeki analizlerin) çökmemesi için 
            # güvenli (fallback) bir durum oluşturup akışa devam ediyoruz.
            state_out = ThermodynamicState(state_in.fluid)
            state_out.update('P', self.P_ambient, 'T', state_in.T)
            return state_out

        # Nested function: Difference between Velocity and Speed of Sound for a given expansion pressure
        def velocity_margin(P_guess):
            h_ideal = PropsSI('H', 'P', P_guess, 'S', state_in.s, state_in.fluid)
            h_actual = state_in.h - self.eta_is * (state_in.h - h_ideal)
            dh = state_in.h - h_actual
            V = math.sqrt(2.0 * dh) if dh > 0 else 0.0
            # Fetch local speed of sound via CoolProp
            a = PropsSI('A', 'P', P_guess, 'H', h_actual, state_in.fluid)
            return V - a

        # First test the fully expanded (unchoked) scenario to ambient pressure
        margin_amb = velocity_margin(self.P_ambient)
        
        if margin_amb <= 0:
            # If V <= a at ambient pressure, flow is unchoked
            self.P_exit = self.P_ambient
            self.is_choked = False
        else:
            # If V > a at ambient pressure, flow is choked.
            # Use bisection to find P_exit where V = a (Mach 1).
            self.is_choked = True
            P_high = state_in.P * 0.99  # Just below stagnation pressure
            P_low = self.P_ambient
            
            for _ in range(50):
                P_guess = (P_high + P_low) / 2.0
                margin = velocity_margin(P_guess)
                
                if abs(margin) < 0.05:  # 0.05 m/s precision tolerance
                    break
                
                if margin > 0:
                    # Velocity > speed of sound. Not compressed enough, increase pressure.
                    P_low = P_guess
                else:
                    P_high = P_guess
                    
            self.P_exit = (P_high + P_low) / 2.0

        # Calculate and assign the final state
        h_ideal = PropsSI('H', 'P', self.P_exit, 'S', state_in.s, state_in.fluid)
        h_out_actual = state_in.h - self.eta_is * (state_in.h - h_ideal)
        
        delta_h = state_in.h - h_out_actual
        self.V_exit = math.sqrt(2.0 * delta_h) if delta_h > 0 else 0.0

        state_out = ThermodynamicState(state_in.fluid)
        state_out.update('P', self.P_exit, 'H', h_out_actual)
        return state_out
