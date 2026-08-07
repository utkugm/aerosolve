# AeroSolve 

**AeroSolve** is a modular, high-fidelity, open-source Python framework designed for aero-engine thermodynamic cycle analysis and propulsion simulation.

> **Status:** `v0.1.0-alpha` — Core 1D thermodynamic cycle module (`aerosolve.cycle`) is active. Compressible flow and aerodynamic blade-sizing modules are under active development.

---

##  Key Features

* **Real-Gas Thermodynamics:** Integrated with `CoolProp` (HEOS backend) utilizing state-repetition optimization (`AbstractState`) for rapid thermodynamic property evaluations.
* **Deterministic Shaft Matching:** Implements a LIFO (Last-In, First-Out) stack accounting methodology to dynamically balance work between single/dual spools (Fan/Compressor & HPT/LPT) without requiring iterative non-linear equation solvers.
* **Bisection Nozzle Solver:** Evaluates choked flow conditions (Mach 1.0) and perfect expansion regimes using numerical bisection to accurately account for pressure thrust.
* **ISA Atmosphere Model:** Full troposphere and lower stratosphere ambient modeling up to 20,000 meters altitude.
* **Validation & Visualization Suite:** Includes 7 operational engine scenarios (Turbojet, Turbofan, HALE UAV, UHBR, Supersonic Interceptor) and an automated altitude performance plotting engine.
---

##  Quickstart

### 1. Install Dependencies

```bash
pip install coolprop matplotlib numpy

```

### 2. Execution

Run the 7 built-in validation scenarios:

```bash
python main.py

```

Run the altitude performance sweep and plot charts:

```bash
python visualizer.py

```

---

##  Usage Example

```python
from aerosolve.cycle import FlightSimulator

# Initialize flight point at 10,000 m, Mach 0.8
sim = FlightSimulator(altitude_m=10000, mach=0.8, inlet_area=2.0)

# Build dual-spool high-bypass turbofan layout
sim.build_turbofan(
    fan_pr=1.6,
    core_pr=25.0,
    bpr=8.0,
    T_max=1400.0,
    dual_spool=True
)

# Execute cycle solution & print thrust analysis
sim.calculate()
sim.show_results()

```

---


##  License

Distributed under the MIT License.

