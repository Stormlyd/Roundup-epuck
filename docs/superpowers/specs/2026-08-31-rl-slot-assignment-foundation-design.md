# RL Slot Assignment Foundation Design

## Goal

Add a locally testable foundation for learned pursuit-slot assignment without
allowing a learned policy to command wheel speeds or bypass the existing CBF
safety filter.

This phase provides:

- a stable 144-action slot contract shared by C++ execution and Python training;
- an injectable C++ policy boundary;
- strict learned-output validation and deterministic fallback;
- a dependency-light Python training-environment skeleton;
- tests proving action parity, fallback behavior, and the unchanged final CBF
  command boundary.

It does not provide a trained model, neural-network runtime, online learning,
or powered-hardware validation.

## Chosen Approach

The learned action is one candidate from:

```text
24 triangular-ring headings x 6 pursuer-to-slot permutations = 144 actions
```

This is preferred over the two alternatives:

- Six permutations only: too weak because the current deterministic code
  already exhaustively finds the minimum immediate travel cost.
- Three arbitrary continuous `(x, y)` goals: unnecessarily enlarges the action
  space and makes geometry validation, training, and sim-to-real comparison
  harder.

The ring radius remains deterministic and phase-owned:

- `PURSUIT`: `0.31 m`
- `SURROUND`: `0.24 m`
- `CAPTURE`: `0.1224 m`

The learned policy selects ring orientation and joint robot-slot assignment,
not phase, radius, velocities, or wheel commands.

## Execution Architecture

```text
fresh world state + phase
  -> slot-policy observation
  -> learned action candidate (0..143)
  -> exact action decoder and geometry validator
       -> valid: learned slot goals
       -> invalid/unavailable: deterministic PursuitSlotAssigner fallback
  -> existing driveToward controller
  -> differential-drive conversion and wheel smoothing
  -> final CBF over every connected and activated robot
  -> serial command path
```

The existing deterministic assigner remains the default when no learned policy
is installed. Installing a policy must not change the no-policy behavior.

## C++ Policy Contract

Add a small Qt-free module under `manager/` with these concepts:

```cpp
constexpr int PursuitSlotHeadingCount = 24;
constexpr int PursuitSlotPermutationCount = 6;
constexpr int PursuitSlotActionCount = 144;

struct PursuitSlotObservation
{
    PursuitWorldState world;
    PursuitPhase phase = PursuitPhase::Idle;
    double radius = 0.0;
    int previousAction = -1;
};

class PursuitSlotPolicy
{
public:
    virtual ~PursuitSlotPolicy() = default;
    virtual bool chooseAction(const PursuitSlotObservation& observation,
                              int& action) = 0;
};
```

The policy is synchronous and non-owning. This phase supplies no production
learned implementation; tests inject a small scripted policy. A later ONNX or
process adapter can implement the same interface after its latency budget is
measured.

The action decoder is a pure function. Action `a` maps to:

```text
heading_index = a / 6
permutation_index = a % 6
heading = heading_index * 2*pi/24
```

Permutation ordering is fixed identically in C++ and Python:

```text
[0,1,2], [0,2,1], [1,0,2], [1,2,0], [2,0,1], [2,1,0]
```

## Learned-Output Validation and Fallback

An action is accepted only when:

- the policy call returns success;
- `0 <= action < 144`;
- all input state and generated goal values are finite;
- the generated triangular ring is inside the confirmed
  `1.460 m x 0.914 m` field with the existing `0.057 m` goal margin;
- its phase and radius match the deterministic state-machine request.

Any failure uses the existing deterministic assignment in the same control
update. If deterministic assignment also cannot produce feasible goals, the
existing controller fallback and final CBF behavior remain unchanged.

Policy decisions are sticky within a phase, matching current slot stability.
The policy is called again on a phase transition or when the remembered goals
become infeasible after target motion. This avoids per-tick slot oscillation in
the first implementation.

## Python Training Environment Skeleton

Create a pure-Python module under `rl/` with no PyTorch, Gymnasium, or Webots
dependency. It defines the same 144 actions and provides:

- action encode/decode and goal generation;
- normalized observation encoding for the fixed field;
- a `SlotAssignmentEnv` that accepts an injected transition callback;
- deterministic reward accounting;
- a scripted transition for unit tests only.

The normalized observation contains:

- one-hot `PURSUIT`, `SURROUND`, or `CAPTURE` phase;
- target normalized `(x, y, vx, vy)`;
- each pursuer's normalized `(x, y, sin(yaw), cos(yaw))` in stable hardware-ID
  order;
- previous action encoded as `0.0` for no learned decision and
  `(action + 1) / 144` otherwise, so all 144 actions remain distinguishable.

The transition callback receives current state and decoded goals, and returns
the next state plus these measured terms:

```text
captured, capture_progress_delta, elapsed_seconds, path_length_metres,
crossings, cbf_interventions, invalid_transition
```

The initial reward is configurable and defaults to:

```text
+100 * captured
+10  * capture_progress_delta
-1   * elapsed_seconds
-2   * path_length_metres
-5   * crossings
-2   * cbf_interventions
-1   * assignment_switch
-100 * invalid_transition
```

These are offline baseline weights, not hardware-calibrated constants. Keeping
the transition callback separate allows the same environment contract to be
driven later by Webots rollouts without embedding a second robot simulator in
this repository.

## Data and Safety Boundaries

- Training happens offline; the real robots never update policy weights.
- A learned policy never selects active participants, target identity, mission
  phase, raw wheel commands, or CBF parameters.
- Invalid, unavailable, or later time-limited inference falls back to the
  deterministic slot assigner.
- Every resulting wheel command still passes through the existing final CBF.
- The current `d_min = 0.100 m`, `r_safe = 0.050 m`, freshness `<100 ms`, and
  snapshot skew `<=50 ms` remain unchanged.
- No result from the Python skeleton authorizes powered movement.

## Tests and Acceptance

Qt-free C++ tests must prove:

- exact action decoding at `0`, every permutation boundary, and `143`;
- rejection of `-1`, `144`, non-finite input, and out-of-field candidates;
- a valid scripted policy changes slot choice while commands still reach the
  final CBF;
- invalid or unavailable scripted policy output exactly matches deterministic
  fallback behavior;
- phase persistence prevents per-tick action switching;
- existing deterministic, pursuit, receiver, coordinate, and CBF tests remain
  green.

Python standard-library tests must prove:

- all 144 actions encode/decode round-trip;
- C++ and Python permutation order and heading formula match fixed fixtures;
- observation length/order/normalization are stable;
- invalid actions and transitions fail closed;
- each reward term has the documented sign and assignment switching is
  penalized.

The implementation is complete only after the full C++ core command, Python
tests, sanitizer run, and `git diff --check` pass locally. Windows Qt build,
Webots training, trained-policy evaluation, and hardware tests remain separate
later gates.
