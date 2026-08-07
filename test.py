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