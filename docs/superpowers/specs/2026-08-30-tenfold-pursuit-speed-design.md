# Tenfold Pursuit Speed Design

## Goal

Increase all commanded linear speeds in the three-stage pursuit test to ten
times their current values because the physical robots only creep at the
existing settings.

## Design

Only the phase-specific linear-speed ceilings change. Pursuer ceilings become
0.200, 0.170, and 0.140 m/s for PURSUIT, SURROUND, and CAPTURE. Target ceilings
become 0.080 m/s normally in PURSUIT, 0.100 m/s under nearby pressure, 0.070 m/s
in SURROUND, and 0.050 m/s before the existing capture-pressure scale is
applied. Turning logic, geometry, role assignment, feedback timeout, ±1000
wheel saturation, wheel slew limiting, and three-cycle zero-speed stop remain
unchanged.

## Verification

Update the real controller behavior test first so the old implementation fails,
then update production constants and documentation. Run the complete core test
binary and a fresh Qt build.

