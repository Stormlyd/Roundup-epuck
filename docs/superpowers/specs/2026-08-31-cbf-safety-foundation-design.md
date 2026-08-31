# ZooidManager CBF Safety Foundation Design

**Date:** 2026-08-31

## Goal

Replace the pursuit controller's final heuristic collision stop with a
strategy-independent Control Barrier Function (CBF) safety layer. The first
stage is locally testable without Qt or hardware and uses the confirmed
`1.460 m x 0.914 m` drivable rectangle. It must fail closed before any wheel
command reaches the existing hardware encoder.

This stage establishes a software-model safety invariant. It does not claim a
physical no-collision guarantee until robot size, localization error, command
latency, braking, and the undocumented speed protocol have been measured on
the Windows hardware stack.

## Fixed Decisions and Assumptions

- World coordinates use metres, `x` to the right and `y` upward.
- The drivable rectangle is `[0, 1.460] x [0, 0.914]`.
- Raw localization endpoints remain `X=63..960` and `Y=229..795`.
- Raw/world mappings are:

  - `x = map(rawX, 63, 960, 1.460, 0)`
  - `y = map(rawY, 229, 795, 0, 0.914)`
  - `rawX = map(x, 1.460, 0, 63, 960)`
  - `rawY = map(y, 0, 0.914, 229, 795)`

- Offline CBF tests use `d_min = 0.100 m` and `r_safe = 0.050 m`.
- Offline freshness uses `T_fresh = 100 ms`; the maximum timestamp spread in
  one safety snapshot is `50 ms`.
- Current wheel conversion remains `1000 command units/(m/s)` for model tests.
  This is a calibration parameter, not a measured physical fact.
- The safety layer sees every connected, activated robot. Mission participants
  receive their nominal commands; other robots receive nominal zero commands
  and remain collision constraints.
- The existing `controlMode=1`, `wheel+2007`, type `0x06` encoder is not treated
  as validated. This stage may test bytes but must not authorize powered motion.

## Approaches Considered

### 1. Extend the existing `distance < 0.14 m` stop heuristic

This is the smallest edit but cannot express boundary constraints, coupled
motion, stale-state handling, or a verifiable safe set. It is rejected.

### 2. Add a pure C++ centralized CBF kernel at the final command boundary

This is the selected approach. It filters the final slew-limited wheel
commands immediately before `ZooidTestMode::update()` returns them to
`ZooidManager`. The kernel has no Qt or solver dependency and is reusable when
LLM mission logic or reinforcement-learning slot assignment is added later.

### 3. Add a general QP package such as OSQP

This provides a mature optimizer but adds a build and deployment dependency to
an old 32-bit Qt/MinGW application before the hardware protocol is validated.
It is deferred unless the small in-tree solver fails measured timing or
robustness requirements.

## Architecture

The execution chain becomes:

```text
receiver bytes
  -> validated status message with host arrival time
  -> canonical raw/world coordinate conversion
  -> one coherent robot-state snapshot
  -> pursuit / future RL / future LLM nominal commands
  -> slew limiting
  -> centralized CBF safety filter
  -> fail-closed validation
  -> existing protocol encoder (offline until validated)
```

The CBF kernel is the final authority over movement. Higher-level code may
choose goals, roles, slots, or nominal wheel speeds, but cannot bypass it.

## Input Integrity

### Message contract

Each parsed message retains:

- message type and sender;
- exact payload length;
- an owned payload copy;
- host monotonic arrival time recorded when the complete frame is accepted;
- a monotonically increasing host receive sequence.

`TYPE_STATUS` is accepted only with exactly eight payload bytes. Its fields are
decoded explicitly as little-endian values rather than copied into a native C++
structure. Bad length, trailer, sender ID, or raw coordinate range cannot
refresh a robot's freshness timestamp.

### Queue contract

The receiver queue is FIFO. Processing `A, B, C` must leave state `C`, never
state `A`. A control cycle consumes all pending messages and retains the newest
valid observation per robot. The manager uses the receive timestamp embedded
in the message; it does not replace it with the later processing time.

### Freshness contract

If any active safety-constrained robot is stale, has a future timestamp, has a
non-finite pose, lies outside the raw calibration range, or makes the snapshot
timestamp spread exceed `50 ms`, the safety layer returns zero for every active
robot and latches a safety fault. Recovery requires a manual restart after
valid feedback resumes.

## Canonical Coordinate Layer

A small Qt-free coordinate module owns all raw/world conversion. The manager,
tests, and position-command path call the same functions. This removes the
current mismatched Y direction and the negative-Y clamp bug.

The conversion tests cover four corners, centre, out-of-range rejection, and
`raw -> world -> raw` round trips with at most one raw count of quantization
error. UI conversion remains a display-only `screenY = H - worldY` operation.

## CBF Model

For robot `i`, the planar unicycle centre dynamics used by the barrier are:

```text
p_dot_i = e(theta_i) * v_i
e(theta_i) = [cos(theta_i), sin(theta_i)]
```

The angular command remains nominal. The CBF modifies the common-mode wheel
component, which is the robot's forward velocity, after slew limiting. The
differential component is preserved unless the filter fails closed.

### Pairwise barriers

For every active robot pair:

```text
h_ij = ||p_i - p_j||^2 - d_min^2
```

The continuous-time CBF constraint is:

```text
2 (p_i - p_j)^T (e_i v_i - e_j v_j) + gamma h_ij >= 0
```

All active pairs are constrained, including the target and stopped extra
robots. For four robots this creates six pairwise constraints.

### Boundary barriers

For every robot centre:

```text
h_left   = x - r_safe
h_right  = field_width - r_safe - x
h_bottom = y - r_safe
h_top    = field_height - r_safe - y
```

Each uses `h_dot + gamma h >= 0`. The initial offline gain is `gamma = 4.0`.

### Command projection

The kernel minimizes squared deviation from the nominal common-mode wheel
commands subject to all pairwise, boundary, non-reverse, and wheel-limit
halfspaces. Because the current problem is small and convex, an in-tree
deterministic projection solver is sufficient. It has a fixed iteration cap,
then independently rechecks every constraint.

Zero common-mode velocity is feasible whenever the current state is already
inside the safe set. If the initial state is outside the set, input is invalid,
the solver does not converge, a residual is non-finite, or post-checking fails,
all wheel commands become exactly zero and the mission receives a latched
`SafetyViolation` fault.

No smoother, clamp, or strategy output may alter a command after this filter.

## Failure Handling

The following conditions stop all active robots:

- stale, future-dated, skewed, non-finite, or out-of-bounds state;
- current pair distance below `d_min`;
- current centre outside the safe boundary;
- malformed status payload or illegal robot ID;
- CBF infeasibility, non-convergence, NaN, or residual violation;
- receiver or command-write failure reported to the mission layer.

The fault is latched. A single good frame cannot restart motion. Stop commands
are sent through the same existing command path, while the absence of a robot
acknowledgement and firmware watchdog remains an explicit hardware limitation.

## Code Boundaries

- `manager/ZooidCoordinates.h/.cpp`: Qt-free canonical coordinate conversion.
- `manager/ZooidCbfSafety.h/.cpp`: Qt-free safety configuration, constraint
  construction, projection, verification, and fail-closed result.
- `manager/ZooidMessage.h/.cpp`: payload ownership, length, arrival time, and
  host sequence.
- `manager/ZooidReceiver.cpp`: accept fragmented reads, preserve FIFO order,
  stamp complete messages, and reject malformed frames.
- `manager/ZooidManager.cpp`: explicit status decoding, canonical coordinate
  calls, and receive-time freshness propagation.
- `manager/ZooidPursuitTypes.h`: confirmed field defaults and safety fault.
- `manager/ZooidTestMode.cpp`: one CBF call after extra robots are assigned zero
  nominal commands and before commands are returned for encoding.
- `ZooidManager.pro`: include the two new production modules.
- `tests/zooid_core_tests.cpp`: all local regression and CBF tests.

No UI redesign, ROS integration, reinforcement-learning policy, LLM control,
new database, or new optimization dependency is part of this stage.

## Verification

### Locally executable on this Mac

The Qt-free core test binary is compiled with Apple Clang in C++14 mode using
`-Wall -Wextra -pedantic`. Tests must demonstrate:

- confirmed field defaults `1.460 x 0.914`;
- coordinate endpoints, midpoint, rejection, and round trip;
- every boundary at `r_safe +/- epsilon`;
- all six four-robot pairs at `d_min +/- epsilon`;
- simultaneous corner and pair constraints;
- permutation-invariant results;
- nominal commands unchanged when constraints are inactive;
- unsafe, stale, skewed, NaN, and solver-failure inputs produce all-zero output;
- one stale robot zeros every active robot;
- existing pursuit, role, state-machine, and stop tests remain green.

Every production behavior is introduced test-first and observed failing before
its implementation is added.

### Not verifiable on this Mac

The complete application requires Qt 5, Windows APIs, COM ports, and the
vendor hardware stack. This Mac currently has Apple Clang but no CMake, qmake,
Qt, Ninja, or pkg-config. Therefore local success can prove only the pure C++
safety core and source-level integration.

Before powered motion, the Windows target must additionally pass:

1. Qt 5.9.7 MinGW32 qmake compilation with no ABI-size assumptions in status
   decoding.
2. Golden-byte checks or a serial capture confirming `controlMode=1`, type
   `0x06`, and the `+2007` wheel encoding against the installed firmware.
3. Suspended-wheel zero, sign, saturation, and watchdog tests.
4. One-robot corner/axis checks, then one-robot ground braking tests.
5. Two-robot low-speed approach tests before enabling four robots.

## Physical Safety Calibration Gate

The provisional values are replaced before a physical no-collision claim:

```text
r_safe >= r_body + e_pos + v_max T_total + v_max^2 / (2 a_brake)

d_min >= 2 r_body + 2 e_pos + 2 v_max T_total + v_max^2 / a_brake
```

Here `r_body` is the maximum installed robot radius, `e_pos` the worst measured
position error, `T_total` the measured end-to-end delay bound, and `a_brake`
the minimum measured braking deceleration. If any bound is unavailable, the
result remains an offline software-model safety result rather than a physical
guarantee.
