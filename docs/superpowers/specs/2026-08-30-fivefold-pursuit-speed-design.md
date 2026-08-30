# Fivefold Pursuit Speed Design

## Goal

Set all phase-specific pursuit-test linear speeds to five times the original
values, which is one half of the current tenfold configuration.

## Design

Pursuer ceilings are 0.100, 0.085, and 0.070 m/s for PURSUIT, SURROUND, and
CAPTURE. Target ceilings are 0.040 m/s normally in PURSUIT, 0.050 m/s under
nearby pressure, 0.035 m/s in SURROUND, and 0.025 m/s before capture-pressure
scaling. Geometry, angular control, ID assignment, ±1000 saturation, slew
limiting, feedback checks, and safe stop are unchanged.

## Verification

Change real controller behavior tests first and observe failure against the
tenfold implementation. Then update production constants and documentation,
run the complete core suite, and rebuild the Qt executable.

