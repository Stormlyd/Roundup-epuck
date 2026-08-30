# ZooidManager Three-Stage Pursuit Test Design

**Date:** 2026-08-30

## Goal

Port the deterministic three-stage pursuit experiment from the local
`epuck-pursuit-webots` repository into ZooidManager as the only test mode. The
runtime must control one moving target and three pursuers through the existing
USB receiver and speed-control frame, without ROS, Webots, reinforcement
learning, Voronoi control, or obstacle planning.

## Scope

The mode provides this mission sequence:

`IDLE -> PURSUIT -> SURROUND -> CAPTURE -> CAPTURED`

`CAPTURED` is the terminal result rather than a fourth motion stage. All four
robots stop when capture is confirmed. The existing Start Test and Stop Test
buttons remain the operator controls. Existing legacy planning modes remain
disabled while this test mode runs.

## Runtime Role Assignment

Robot hardware IDs are discovered from receiver feedback and are not assumed
to be 0, 1, 2, and 3.

At Start Test:

1. Take a snapshot of active robots with fresh feedback.
2. Sort them by their real Zooid ID.
3. Assign the lowest ID to `target`.
4. Assign the next three IDs to `pursuer_1`, `pursuer_2`, and `pursuer_3`.
5. Keep any additional robots stopped.

The role map is frozen until Stop Test or a terminal safety fault. A robot that
appears later cannot silently replace a missing participant. The UI displays
the four role-to-ID assignments for the current run.

Starting is rejected unless at least four robots have fresh feedback. A
participant that disconnects or becomes stale terminates the mission and
causes a safe stop.

## World State

Every accepted receiver update supplies each participant's real ID, planar
position, heading, and update freshness. ZooidManager's mapped `Vector2`
coordinates are treated as metres. The receiver heading convention
(`0 degrees = up`, clockwise positive) is converted to the mathematical
controller convention (`0 radians = +x`, counter-clockwise positive):

`yaw = radians(90 - receiver_orientation_degrees)`

Target planar velocity is estimated from consecutive fresh position samples
and bounded before it is used for short-horizon target prediction. Duplicate
or stale observations do not advance state-machine dwell counters.

The controller runs at the existing 100 ms manager cadence. A participant is
stale when it has no valid feedback for 500 ms.

## Three-Stage State Machine

The hardware-table profile from `epuck-pursuit-webots` is the starting
parameter set.

### PURSUIT

- Predict the target pose over a short horizon using its bounded measured
  velocity.
- Build three equally spaced candidate slots on a `0.31 m` pursuit hold ring.
- Choose the slot permutation with minimum total squared travel distance.
- Preserve slot bearings while the phase is active to avoid assignment swaps.
- Never command a pursuer outward merely to reach the nominal ring; its goal
  radius is capped by its current target distance.
- Limit pursuer linear speed to `0.020 m/s`.

Transition readiness requires all three pursuers to be within `0.36 m` of the
target, a feasible ring inside the field, at least 35 fresh phase ticks, and
three consecutive ready observations.

### SURROUND

- Assign a persistent triangular ring around the target at radius `0.24 m`.
- Select a feasible ring heading and the minimum-cost pursuer-to-slot
  permutation.
- Keep the non-expanding radial goal behavior during convergence.
- Limit pursuer linear speed to `0.017 m/s`.

Transition readiness requires every pursuer radius to be within `0.055 m` of
the surround radius, circular angular gaps within the original hardware
geometry limits, at least 35 fresh phase ticks, and three consecutive ready
observations.

If any pursuer remains beyond the `0.46 m` break radius, or no surround ring is
feasible, for eight consecutive observations, transition back to PURSUIT.

### CAPTURE

- Tighten the assigned triangular ring to `0.17 * 0.72 = 0.1224 m`.
- Preserve phase-local slot assignments unless recovery is required.
- Limit pursuer linear speed to `0.014 m/s`.

Containment requires all pursuers within `0.17 m` of the target and circular
angular gaps between the hardware profile limits (`1.65` and `2.60` radians).
Twenty consecutive contained observations produce CAPTURED.

Before containment is first acquired, a true escape or loss of a feasible
capture ring falls back to SURROUND or PURSUIT after the original break and
timeout windows. After acquisition, six consecutive broken observations cause
the same geometry-dependent fallback.

### CAPTURED

Send zero wheel speed to the target and all pursuers, flush the relevant
receiver queues, retain the final role map and result in the UI, and wait for
Stop Test or another explicit Start Test. The post-capture orbit from the
latest simulation runtime is intentionally excluded because this mode has
exactly three motion stages and prioritizes a safe physical completion.

## Target Motion

The target uses the deterministic rule-based evasive behavior from the pursuit
project, adapted to an obstacle-free rectangular field:

- PURSUIT: cruise at `0.008 m/s`, rising to `0.010 m/s` under close pressure.
- SURROUND: cap speed at `0.007 m/s`.
- CAPTURE: cap speed at `0.005 m/s` and reduce it further as more pursuers enter
  the capture-pressure radius.
- CAPTURED and IDLE: zero speed.

Escape direction is chosen away from the pursuer geometry while remaining
inside the ZooidManager field with the hardware boundary margin. The goal is
refreshed periodically. If no safe escape goal exists, the target stops or
rotates in place conservatively; it must never be commanded outside the field.

## Differential-Drive Command Pipeline

For each robot, a point controller produces linear velocity `v` and angular
velocity `omega`. Large heading errors first reduce or eliminate translation.
Pairwise separation and field-boundary guards may only reduce or redirect the
nominal command.

Differential-drive conversion uses:

- `left = v - omega * wheel_base / 2`
- `right = v + omega * wheel_base / 2`

The result is converted to the receiver's signed wheel-speed units, slew-rate
limited, and saturated to `[-1000, 1000]`. It is then passed to the existing
`controlRobotSpeed(real_id, left, right, color)` path, which encodes the signed
values in the established position fields with `controlMode = 1` and sends the
frame to the receiver responsible for that real ID.

Calibration constants remain centralized and named so physical wheel-base and
byte-to-speed calibration can be changed without modifying pursuit geometry.
Initial defaults are conservative and do not command full speed during this
mission.

## Safety and Failure Handling

The controller sends zero speed to all selected participants and ends the run
when any of these occurs:

- Stop Test is clicked.
- A selected participant is missing or stale for 500 ms.
- Receiver transmission reports an error.
- Position, heading, goal, or computed command is non-finite.
- The controller cannot maintain a valid role map.
- The application closes or the test mode is replaced.

Extra online robots are also held at zero while the test runs. Safe-stop sends
are flushed immediately. Repeated Stop Test operations are idempotent. A new
Start Test always rebuilds the mapping from a new online-ID snapshot and resets
all temporal and assignment memory.

## UI Behavior

The test panel retains only:

- Start Test
- Stop Test
- Status text containing the state, role map, target distances, capture
  progress, and the latest transition or fault reason

Start Test is disabled while a mission is active. Stop Test remains available
through all active phases. The UI must not require the operator to know the
random hardware IDs in advance.

## Code Structure

The implementation will separate deterministic logic from Qt and serial I/O:

- pursuit models and geometry utilities;
- role assignment and freshness tracking;
- state machine and transition diagnostics;
- phase goal/slot assignment;
- target and pursuer controllers;
- differential-drive conversion and command smoothing;
- ZooidManager integration and UI status exposure.

Pure components will not depend on QObject, USB devices, or wall-clock calls,
allowing deterministic unit testing.

## Verification

Automated tests cover:

- random and non-contiguous hardware ID assignment;
- rejection with fewer than four fresh robots;
- frozen role maps during a run;
- PURSUIT -> SURROUND -> CAPTURE -> CAPTURED;
- dwell counters advancing only on fresh observations;
- surround and capture fallback paths;
- 120-degree slot geometry and stable minimum-cost assignment;
- coordinate and heading conversion;
- differential-drive signs, saturation, and slew limiting;
- feedback timeout, participant loss, receiver error, and repeated safe stop;
- zero commands for unselected robots and terminal CAPTURED.

Verification also includes the existing C++ core test executable and a clean
Qt MinGW debug build. Hardware motion is not claimed as calibrated solely from
software tests; the generated binary starts with conservative speed limits and
retains immediate Stop Test behavior.

## Acceptance Criteria

The feature is complete when a clean build produces ZooidManager with the two
test buttons, automatically maps four arbitrary online IDs, runs the exact
three motion stages from fresh receiver feedback, sends all commands through
the existing speed frame, visibly reports phase and mapping, safely stops on
capture or any fault, and all deterministic automated tests pass.
