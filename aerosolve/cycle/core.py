# aero/core.py
"""
Core thermodynamic primitives used throughout the aero package.
"""
import logging
import CoolProp.CoolProp as CP
from CoolProp.CoolProp import AbstractState
from enum import Enum, auto

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')


class ThermodynamicState:
    """
    Stores and updates the four primary thermodynamic properties of a
    fluid at a single cycle station.

    All four properties (P, T, h, s) are derived from any two independent
    inputs via CoolProp's HEOS back-end.  The internal AbstractState object
    is reused across updates to avoid repeated object construction overhead.

    Units
    -----
    P : Pa  |  T : K  |  h : J/kg  |  s : J/(kg·K)
    """

    @property
    def rho(self):
        """Real gas density [kg/m³]"""
        if self.P is not None:
            try:
                return self._AS.rhomass()
            except ValueError:
                return None
        return None

    @property
    def a(self):
        """Real gas local speed of sound [m/s]"""
        if self.P is not None:
            try:
                return self._AS.speed_sound()
            except ValueError:
                # CoolProp raises ValueError for speed of sound in two-phase regions
                return None
        return None
    
    
    def __init__(self, fluid="Air"):
        self.fluid = fluid
        self.P = None
        self.T = None
        self.h = None
        self.s = None
        
        # Reuse a single AbstractState instance for speed (4–15× faster than PropsSI)
        self._AS = AbstractState("HEOS", self.fluid)

    def update(self, input1_name, input1_value, input2_name, input2_value):
        """
        Update the state given any two independent thermodynamic properties.

        Parameters
        ----------
        input1_name, input2_name : 'P', 'T', 'H', 'S', or 'D'
        input1_value, input2_value : corresponding values in SI units
        """
        try:
            key_map = {
                'P': CP.iP, # Pressure
                'T': CP.iT, # Temperature
                'H': CP.iHmass, # Enthalpy
                'S': CP.iSmass, # Entropy
                'D': CP.iDmass  # Density
            }
            
            try:
                k1 = key_map[input1_name.upper()]
                k2 = key_map[input2_name.upper()]
            except KeyError as ke:
                raise ValueError(f"Unsupported thermodynamic property: {ke}. Supported keys: P, T, H, S, D")

            # generate_update_pair handles the required CoolProp input ordering
            pair, v1, v2 = CP.generate_update_pair(k1, input1_value, k2, input2_value)
            self._AS.update(pair, v1, v2)

            self.P = self._AS.p()
            self.T = self._AS.T()
            self.h = self._AS.hmass()
            self.s = self._AS.smass()

        except Exception as e:
            logging.error(f"CoolProp update failed for {self.fluid}: {e}")
            raise ValueError(
                f"CoolProp error ({self.fluid}): "
                f"{input1_name}={input1_value}, {input2_name}={input2_value}. "
                f"Detail: {e}"
            )

    def __repr__(self):
        if self.P is None:
            return f"[{self.fluid}] — uninitialized"
        return (
            f"P: {self.P/1e5:5.2f} bar | "
            f"T: {self.T:7.2f} K | "
            f"h: {self.h/1e3:8.2f} kJ/kg | "
            f"s: {self.s/1e3:7.4f} kJ/(kg·K)"
        )


class EnergyRole(Enum):
    """
    Thermodynamic role of each component in the engine cycle.

    Used by AeroEngine to automate shaft-power matching (1st Law balancing)
    without requiring the user to manually wire compressor work to turbines.

    SHAFT_CONSUMER : absorbs shaft work        — Compressor, Fan
    SHAFT_PRODUCER : delivers shaft work       — Turbine
    HEAT_ADDER     : adds thermal energy       — Combustor, Afterburner
    PASS_THROUGH   : converts energy form only — Diffuser, Nozzle
    """
    SHAFT_CONSUMER = auto()
    SHAFT_PRODUCER = auto()
    HEAT_ADDER     = auto()
    PASS_THROUGH   = auto()