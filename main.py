# main.py
from aerosolve.cycle import FlightSimulator

def run_scenarios():
    # ------------------------------------------------------------------
    # Scenario 1 — Standard turbojet at cruise altitude
    # ------------------------------------------------------------------
    print("\n--- SCENARIO 1: STANDARD TURBOJET (CRUISE) ---")
    tj = FlightSimulator(altitude_m=10000, mach=0.8, inlet_area=1.0)
    tj.build_turbojet(pr=15.0, T_max=1300.0).calculate()
    tj.show_results()

    # ------------------------------------------------------------------
    # Scenario 2 — Modern high-bypass turbofan at cruise altitude
    # ------------------------------------------------------------------
    print("\n--- SCENARIO 2: MODERN TURBOFAN (CRUISE) ---")
    tf = FlightSimulator(altitude_m=10000, mach=0.8, inlet_area=2.0)
    tf.build_turbofan(fan_pr=1.6, core_pr=25.0, bpr=8.0, T_max=1400.0, dual_spool=True).calculate()
    tf.show_results()

    # ------------------------------------------------------------------
    # Scenario 3 — Turbofan: sea-level static (departure roll)
    # ------------------------------------------------------------------
    print("\n--- SCENARIO 3: TURBOFAN (DEPARTURE ROLL) ---")
    sea_level = FlightSimulator(altitude_m=0, mach=0.2, inlet_area=2.5)
    sea_level.build_turbofan(bpr=8.0, T_max=1400.0, dual_spool=True).calculate()
    sea_level.show_results()

    # ------------------------------------------------------------------
    # Scenario 4 — Supersonic Interceptor (Turbojet with Afterburner)
    # ------------------------------------------------------------------
    print("\n--- SCENARIO 4: SUPERSONIC INTERCEPTOR (MACH 2.0) ---")
    # High Mach generates massive Ram Recovery in the diffuser. 
    # Mechanical PR is kept relatively low to avoid exceeding material pressure limits.
    interceptor = FlightSimulator(altitude_m=15000, mach=2.0, inlet_area=0.8)
    interceptor.build_turbojet(pr=10.0, T_max=1600.0, has_afterburner=True).calculate()
    interceptor.show_results()

    # ------------------------------------------------------------------
    # Scenario 5 — High-Altitude Long Endurance (HALE) UAV
    # ------------------------------------------------------------------
    print("\n--- SCENARIO 5: HALE UAV (HIGH ALTITUDE, LOW SPEED) ---")
    # Operating in the lower stratosphere (18km) where density is extremely low.
    # Requires a highly efficient, low-pressure core to maintain stability.
    uav = FlightSimulator(altitude_m=18000, mach=0.5, inlet_area=1.0)
    uav.build_turbofan(fan_pr=1.4, core_pr=15.0, bpr=5.0, T_max=1200.0, dual_spool=True).calculate()
    uav.show_results()

    # ------------------------------------------------------------------
    # Scenario 6 — Next-Gen Ultra-High Bypass (UHBR) Turbofan
    # ------------------------------------------------------------------
    print("\n--- SCENARIO 6: NEXT-GEN UHBR TURBOFAN (BPR 15) ---")
    # Pushing computational boundaries with massive bypass and high core efficiency.
    # This will heavily test the LIFO shaft matching logic (LPT driving the massive Fan).
    uhbr = FlightSimulator(altitude_m=11000, mach=0.85, inlet_area=3.5)
    uhbr.build_turbofan(
        fan_pr=1.3, core_pr=40.0, bpr=15.0, T_max=1700.0, 
        hpc_eta=0.88, hpt_eta=0.92, dual_spool=True
    ).calculate()
    uhbr.show_results()

    # ------------------------------------------------------------------
    # Scenario 7 — Static Ground Run (Test Cell)
    # ------------------------------------------------------------------
    print("\n--- SCENARIO 7: STATIC GROUND RUN (MACH 0.0) ---")
    # Validates the V_eff = 50 m/s numerical floor in the solver to prevent 
    # zero-mass-flow singularities during static testing.
    static_test = FlightSimulator(altitude_m=0, mach=0.0, inlet_area=2.5)
    static_test.build_turbofan(fan_pr=1.5, core_pr=20.0, bpr=8.0, T_max=1300.0, dual_spool=True).calculate()
    static_test.show_results()


if __name__ == "__main__":
    run_scenarios()