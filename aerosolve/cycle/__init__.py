# aero/__init__.py
"""
Aerosolve - High-Fidelity Object-Oriented Thermodynamic Cycle Solver for Jet Engines.
"""

from .flight import FlightSimulator
from .engine import AeroEngine
from .core import ThermodynamicState, EnergyRole
from .components import Diffuser, Compressor, Combustor, Turbine, Nozzle

__all__ = [
    "FlightSimulator",
    "AeroEngine",
    "ThermodynamicState",
    "EnergyRole",
    "Diffuser",
    "Compressor",
    "Combustor",
    "Turbine",
    "Nozzle",
]
