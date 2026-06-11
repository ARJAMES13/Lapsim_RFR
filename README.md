# RFR Lap Time Simulation

A MATLAB lap time simulation workflow built for Raftar Formula Racing’s EV powertrain studies.

The solver estimates speed profile, lap time, acceleration, DC current draw, terminal power, SoC drop and accumulator heat losses over single or multiple laps. It is mainly intended for comparing setup changes such as gearing, mass, aero coefficients, DRS usage and battery limits during vehicle-level design.

---

## Current Setup

The repository is currently configured around the RFR EV powertrain assumptions:

- Motor: EMRAX 228 HV
- Cell: Molicel P45B
- Pack architecture: 128 cells in series
- Vehicle model: rear-driven Formula Student EV

For a different vehicle, the motor map, current estimation model, battery parameters and vehicle constants should be updated before using the results for design decisions.

---

## Files

```text
lapsim_rfr/
│
├── build_motor_map.m
├── lap_time_simulation_multilap_function.m
├── postprocess_lap_simulation_terminal_only.m
├── fsg_track_left_right_data.xlsx
├── emrax228_hvcc_torque_map.mat
├── lap_solver_log.mat
├── lapsim.pdf
└── .gitattributes
