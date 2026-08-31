# RL Slot Assignment Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a tested, injectable 144-action learned slot-assignment boundary with deterministic fallback and a pure-Python offline environment skeleton, while leaving wheel control and final CBF authority unchanged.

**Architecture:** A Qt-free C++ policy contract decodes `24 x 6` discrete actions. `PursuitSlotAssigner` accepts an optional non-owning policy, validates its candidate, and otherwise executes its existing deterministic search. A standard-library Python module mirrors the action/observation/reward contract and delegates dynamics to an injected transition callback.

**Tech Stack:** C++14 standard library, existing Qt-free core test binary, Python 3.9 standard library `unittest`.

**Repository rule:** The user requested local uncommitted work. Do not create commits, push, merge, or run hardware/serial commands. Replace commit checkpoints with `git diff --check` and `git status --short`.

---

## File Map

- Create `manager/ZooidPursuitSlotPolicy.h`: policy interface, action constants, and decoder declaration.
- Create `manager/ZooidPursuitSlotPolicy.cpp`: fixed permutation table and exact `0..143` decoder.
- Modify `manager/ZooidPursuitGeometry.h/.cpp`: optional policy injection, learned-candidate validation, phase persistence, and unchanged deterministic fallback.
- Modify `manager/ZooidTestMode.h`: expose a testable policy setter without changing production default behavior.
- Modify `ZooidManager.pro`: add the new Qt-free C++ source/header.
- Modify `tests/zooid_core_tests.cpp`: action, learned-policy, fallback, persistence, and final-CBF regressions.
- Create `rl/__init__.py`: package marker and public exports.
- Create `rl/slot_assignment_env.py`: mirrored action contract, observation encoding, reward accounting, and transition-driven environment.
- Create `rl/test_slot_assignment_env.py`: Python standard-library tests.
- Create `rl/README.md`: offline-only usage and safety boundary.
- Modify `docs/test-mode-operation.md`: record that no trained policy/runtime is installed and CBF remains final.

### Task 1: Add the shared C++ action contract

**Files:**
- Create: `manager/ZooidPursuitSlotPolicy.h`
- Create: `manager/ZooidPursuitSlotPolicy.cpp`
- Modify: `tests/zooid_core_tests.cpp`

- [ ] **Step 1: Write failing action-decoder tests**

Add the new header include and a test that checks invalid values, all 144
round-trips, and the fixed boundary fixtures:

```cpp
#include "../manager/ZooidPursuitSlotPolicy.h"

static void testPursuitSlotActionContract()
{
    int heading = -1;
    std::array<int, 3> permutation{{-1, -1, -1}};
    if (decodePursuitSlotAction(-1, heading, permutation) ||
        decodePursuitSlotAction(PursuitSlotActionCount, heading, permutation))
    {
        std::cerr << "slot-invalid-action-accepted failed\n";
        ++failures;
    }

    const std::array<std::array<int, 3>, 6> expected{{
        {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
        {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}
    }};
    for (int action = 0; action < PursuitSlotActionCount; ++action)
    {
        if (!decodePursuitSlotAction(action, heading, permutation) ||
            heading != action / PursuitSlotPermutationCount ||
            permutation != expected[action % PursuitSlotPermutationCount])
        {
            std::cerr << "slot-action-round-trip failed\n";
            ++failures;
            return;
        }
    }
}
```

- [ ] **Step 2: Run RED and verify the missing contract is the failure**

Run the full core compile command from Task 4. Expected: compilation fails
because `ZooidPursuitSlotPolicy.h` does not exist.

- [ ] **Step 3: Implement the minimal contract**

Create the header:

```cpp
#ifndef ZOOIDPURSUITSLOTPOLICY_H
#define ZOOIDPURSUITSLOTPOLICY_H

#include "ZooidPursuitTypes.h"

#include <array>

constexpr int PursuitSlotHeadingCount = 24;
constexpr int PursuitSlotPermutationCount = 6;
constexpr int PursuitSlotActionCount =
    PursuitSlotHeadingCount * PursuitSlotPermutationCount;

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

bool decodePursuitSlotAction(int action,
                             int& headingIndex,
                             std::array<int, 3>& permutation);

#endif // ZOOIDPURSUITSLOTPOLICY_H
```

Create the implementation:

```cpp
#include "ZooidPursuitSlotPolicy.h"

bool decodePursuitSlotAction(int action,
                             int& headingIndex,
                             std::array<int, 3>& permutation)
{
    static const std::array<std::array<int, 3>, 6> permutations{{
        {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
        {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}
    }};
    if (action < 0 || action >= PursuitSlotActionCount)
        return false;
    headingIndex = action / PursuitSlotPermutationCount;
    permutation = permutations[static_cast<std::size_t>(
        action % PursuitSlotPermutationCount)];
    return true;
}
```

Do not add model loading, confidence, asynchronous calls, or dependencies.

- [ ] **Step 4: Run GREEN**

Compile with `manager/ZooidPursuitSlotPolicy.cpp` added to the core command and
run the binary. Expected: `zooid core tests passed`.

- [ ] **Step 5: Local checkpoint without commit**

Run:

```bash
git diff --check
git status --short
```

### Task 2: Inject learned actions with deterministic fallback

**Files:**
- Modify: `manager/ZooidPursuitGeometry.h`
- Modify: `manager/ZooidPursuitGeometry.cpp`
- Modify: `manager/ZooidTestMode.h`
- Modify: `tests/zooid_core_tests.cpp`
- Modify: `ZooidManager.pro`

- [ ] **Step 1: Write failing policy and fallback tests**

Add a real scripted policy:

```cpp
class ScriptedSlotPolicy : public PursuitSlotPolicy
{
public:
    bool succeed = true;
    int action = 0;
    int calls = 0;

    bool chooseAction(const PursuitSlotObservation&, int& selected) override
    {
        ++calls;
        selected = action;
        return succeed;
    }
};
```

Extend the scripted policy with `int seenPreviousAction = -2` and store
`observation.previousAction` on every call. Add these concrete tests:

```cpp
static bool sameGoals(const std::array<PursuitPose, 3>& first,
                      const std::array<PursuitPose, 3>& second)
{
    for (std::size_t i = 0; i < first.size(); ++i)
        if (!near(first[i].x, second[i].x) ||
            !near(first[i].y, second[i].y)) return false;
    return true;
}

static void testLearnedSlotPolicyAndDeterministicFallback()
{
    const PursuitWorldState world = makeGeometryWorld();
    ScriptedSlotPolicy learned;
    PursuitSlotAssigner learnedAssigner;
    learnedAssigner.setPolicy(&learned);
    std::array<PursuitPose, 3> learnedGoals;
    if (!learnedAssigner.assign(
            world, PursuitPhase::Pursuit, 0.31, false, learnedGoals)) {
        std::cerr << "learned-slot-action-rejected failed\n";
        ++failures;
        return;
    }
    const auto ring = makeTriangularRing(world.target.pose, 0.31, 0.0);
    if (!sameGoals(learnedGoals, ring)) {
        std::cerr << "learned-slot-action-mismatch failed\n";
        ++failures;
    }

    for (int invalid : {-1, PursuitSlotActionCount}) {
        ScriptedSlotPolicy bad;
        bad.action = invalid;
        PursuitSlotAssigner withFallback;
        withFallback.setPolicy(&bad);
        PursuitSlotAssigner baseline;
        std::array<PursuitPose, 3> fallbackGoals;
        std::array<PursuitPose, 3> baselineGoals;
        if (!withFallback.assign(world, PursuitPhase::Pursuit, 0.31,
                                 false, fallbackGoals) ||
            !baseline.assign(world, PursuitPhase::Pursuit, 0.31,
                             false, baselineGoals) ||
            !sameGoals(fallbackGoals, baselineGoals)) {
            std::cerr << "invalid-policy-did-not-fallback failed\n";
            ++failures;
        }
    }
}

static void testLearnedSlotPolicyIsPhaseStickyAndClearable()
{
    ScriptedSlotPolicy policy;
    PursuitSlotAssigner assigner;
    assigner.setPolicy(&policy);
    PursuitWorldState world = makeGeometryWorld();
    std::array<PursuitPose, 3> goals;
    assigner.assign(world, PursuitPhase::Pursuit, 0.31, false, goals);
    world.target.pose.x += 0.01;
    assigner.assign(world, PursuitPhase::Pursuit, 0.31, false, goals);
    if (policy.calls != 1) {
        std::cerr << "learned-policy-not-phase-sticky failed\n";
        ++failures;
    }
    assigner.assign(world, PursuitPhase::Surround, 0.24, false, goals);
    if (policy.calls != 2) {
        std::cerr << "learned-policy-not-called-on-phase-change failed\n";
        ++failures;
    }
    assigner.clear();
    assigner.assign(world, PursuitPhase::Surround, 0.24, false, goals);
    if (policy.calls != 3 || policy.seenPreviousAction != -1) {
        std::cerr << "learned-policy-clear-contract failed\n";
        ++failures;
    }
}

static void testLearnedSlotPolicyCannotBypassMissionCbf()
{
    ScriptedSlotPolicy policy;
    ZooidTestMode mode;
    mode.setSlotPolicy(&policy);
    auto fleet = testFleet(1000);
    for (auto& robot : fleet)
        if (robot.id == 29) robot.pose.x = 0.049;
    if (!mode.start(fleet, 1000)) {
        std::cerr << "learned-cbf-start failed\n";
        ++failures;
        return;
    }
    const PursuitControlOutput output = mode.update(
        fleet, 1050, 1.460, 0.914, 1);
    if (output.fault != PursuitFault::SafetyViolation ||
        !hasExactZeroCommandsForActiveFleet(fleet, output.commands)) {
        std::cerr << "learned-policy-bypassed-cbf failed\n";
        ++failures;
    }
}
```

Append these cases to `testLearnedSlotPolicyAndDeterministicFallback`:

```cpp
    ScriptedSlotPolicy unavailable;
    unavailable.succeed = false;
    PursuitSlotAssigner unavailableAssigner;
    unavailableAssigner.setPolicy(&unavailable);
    PursuitSlotAssigner unavailableBaseline;
    std::array<PursuitPose, 3> unavailableGoals;
    std::array<PursuitPose, 3> unavailableBaselineGoals;
    if (!unavailableAssigner.assign(world, PursuitPhase::Pursuit, 0.31,
                                    false, unavailableGoals) ||
        !unavailableBaseline.assign(world, PursuitPhase::Pursuit, 0.31,
                                    false, unavailableBaselineGoals) ||
        !sameGoals(unavailableGoals, unavailableBaselineGoals)) {
        std::cerr << "unavailable-policy-did-not-fallback failed\n";
        ++failures;
    }

    PursuitWorldState edge = world;
    edge.target.pose.x = 0.35;
    ScriptedSlotPolicy infeasible;
    infeasible.action = 72;
    PursuitSlotAssigner infeasibleAssigner;
    infeasibleAssigner.setPolicy(&infeasible);
    PursuitSlotAssigner edgeBaseline;
    std::array<PursuitPose, 3> infeasibleGoals;
    std::array<PursuitPose, 3> edgeBaselineGoals;
    if (!infeasibleAssigner.assign(edge, PursuitPhase::Pursuit, 0.31,
                                   false, infeasibleGoals) ||
        !edgeBaseline.assign(edge, PursuitPhase::Pursuit, 0.31,
                             false, edgeBaselineGoals) ||
        !sameGoals(infeasibleGoals, edgeBaselineGoals)) {
        std::cerr << "infeasible-policy-did-not-fallback failed\n";
        ++failures;
    }
```

- [ ] **Step 2: Run RED**

Run the core command. Expected: compilation fails because `setPolicy` is not
defined.

- [ ] **Step 3: Add the smallest policy boundary**

Extend `PursuitSlotAssigner` with:

```cpp
void setPolicy(PursuitSlotPolicy* policy);

private:
    PursuitSlotPolicy* policy_ = nullptr;
    int previousAction_ = -1;
```

Add this adapter-facing pass-through to `ZooidTestMode`:

```cpp
void setSlotPolicy(PursuitSlotPolicy* policy) { slots_.setPolicy(policy); }
```

In `PursuitSlotAssigner::assign`, keep remembered feasible goals first. When a
new choice is required:

1. Call the installed policy once with world, phase, radius, and previous
   action.
2. Decode its action using `decodePursuitSlotAction`.
3. Build the existing triangular ring at
   `headingIndex * 2*pi/PursuitSlotHeadingCount`.
4. Apply the decoded permutation by pursuer index.
5. Accept only finite goals inside the existing `0.057 m` margin.
6. Otherwise execute the pre-existing 24-heading/minimum-distance search.
7. Store the chosen bearings and action only after a valid choice.

Refactor the deterministic loop only enough to record its equivalent action;
preserve sample order, strict `<` tie behavior, phase persistence, and
non-expanding radius behavior.

`clear()` resets phase memory and `previousAction_`, but does not clear the
non-owning `policy_` pointer.

- [ ] **Step 4: Add qmake sources**

Add `manager/ZooidPursuitSlotPolicy.cpp` to `SOURCES` and
`manager/ZooidPursuitSlotPolicy.h` to `HEADERS`. Do not add ML libraries.

- [ ] **Step 5: Run GREEN and regressions**

Run the full core test binary. Expected: `zooid core tests passed` with no
warnings under `-Werror`.

- [ ] **Step 6: Local checkpoint without commit**

Run `git diff --check` and inspect only the Task 2 diff.

### Task 3: Add the dependency-free Python environment skeleton

**Files:**
- Create: `rl/__init__.py`
- Create: `rl/slot_assignment_env.py`
- Create: `rl/test_slot_assignment_env.py`
- Create: `rl/README.md`

- [ ] **Step 1: Write failing Python contract tests**

Use `unittest` and require:

```python
def test_all_actions_round_trip(self):
    for action in range(ACTION_COUNT):
        heading, permutation = decode_action(action)
        self.assertEqual(heading, action // 6)
        self.assertEqual(permutation, PERMUTATIONS[action % 6])

def test_previous_action_encoding_is_unambiguous(self):
    self.assertEqual(encode_previous_action(-1), 0.0)
    self.assertEqual(encode_previous_action(0), 1.0 / 144.0)
    self.assertEqual(encode_previous_action(143), 1.0)
```

Use this shared fixture and these public-behavior tests:

```python
def sample_state(previous_action=-1):
    return SlotState(
        phase="PURSUIT",
        target=RobotState(0.730, 0.457, 0.0),
        target_vx=0.03,
        target_vy=-0.03,
        pursuers=(
            RobotState(0.365, 0.2285, 0.0),
            RobotState(0.730, 0.457, math.pi / 2.0),
            RobotState(1.095, 0.6855, math.pi),
        ),
        previous_action=previous_action,
    )

def test_observation_layout(self):
    observation = encode_observation(sample_state())
    self.assertEqual(len(observation), 20)
    expected = (
        1.0, 0.0, 0.0,
        0.5, 0.5, 1.0, -1.0,
        0.25, 0.25, 0.0, 1.0,
        0.5, 0.5, 1.0, 0.0,
        0.75, 0.75, 0.0, -1.0,
        0.0,
    )
    for actual, wanted in zip(observation, expected):
        self.assertAlmostEqual(actual, wanted)

def test_invalid_action_terminates(self):
    calls = []
    env = SlotAssignmentEnv(lambda state, goals: calls.append((state, goals)))
    original = env.reset(sample_state())
    observation, reward, terminated, info = env.step(-1)
    self.assertEqual(observation, original)
    self.assertEqual(reward, -100.0)
    self.assertTrue(terminated)
    self.assertEqual(info, {"invalid": True})
    self.assertEqual(calls, [])

def test_reward_terms_and_switch_penalty(self):
    self.assertEqual(score_reward(RewardTerms(captured=True), False), 100.0)
    self.assertEqual(score_reward(
        RewardTerms(capture_progress_delta=1.0), False), 10.0)
    self.assertEqual(score_reward(
        RewardTerms(elapsed_seconds=1.0), False), -1.0)
    self.assertEqual(score_reward(
        RewardTerms(path_length_metres=1.0), False), -2.0)
    self.assertEqual(score_reward(RewardTerms(crossings=1), False), -5.0)
    self.assertEqual(score_reward(
        RewardTerms(cbf_interventions=1), False), -2.0)
    self.assertEqual(score_reward(RewardTerms(), True), -1.0)
    self.assertEqual(score_reward(
        RewardTerms(invalid_transition=True), False), -100.0)

def test_successful_transition(self):
    def transition(state, goals):
        self.assertEqual(len(goals), 3)
        return state, RewardTerms(capture_progress_delta=0.5), False

    env = SlotAssignmentEnv(transition)
    env.reset(sample_state())
    observation, reward, terminated, info = env.step(0)
    self.assertAlmostEqual(observation[-1], 1.0 / 144.0)
    self.assertEqual(reward, 5.0)
    self.assertFalse(terminated)
    self.assertEqual(info, {"invalid": False})
```

- [ ] **Step 2: Run RED**

Run:

```bash
/usr/bin/python3 -m unittest -v rl.test_slot_assignment_env
```

Expected: import failure because the module does not exist.

- [ ] **Step 3: Implement state and action primitives**

Use immutable dataclasses:

```python
@dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    yaw: float

@dataclass(frozen=True)
class SlotState:
    phase: str
    target: RobotState
    target_vx: float
    target_vy: float
    pursuers: Tuple[RobotState, RobotState, RobotState]
    previous_action: int = -1
```

Define fixed field, margin, phase radii, six permutations, `decode_action`,
`goals_for_action`, `encode_previous_action`, validation, and a stable
20-element observation tuple:

```text
3 phase one-hot
4 target x/y/vx/vy
12 pursuer x/y/sin(yaw)/cos(yaw)
1 previous action
```

Normalize positions by `1.460` and `0.914`; clamp target velocity by the
existing `0.03 m/s` estimate bound before dividing by `0.03`.

- [ ] **Step 4: Implement transition-driven step and reward**

Define `RewardTerms`, `RewardWeights`, and:

```python
class SlotAssignmentEnv:
    def __init__(self, transition, weights=None):
        if not callable(transition):
            raise TypeError("transition must be callable")
        self._transition = transition
        self._weights = weights if weights is not None else RewardWeights()
        self._state = None

    def reset(self, state):
        validate_state(state)
        self._state = state
        return encode_observation(state)

    def step(self, action):
        if self._state is None:
            raise RuntimeError("reset must be called before step")
        previous = self._state
        try:
            goals = goals_for_action(previous, action)
        except (TypeError, ValueError):
            return (encode_observation(previous),
                    self._weights.invalid_transition,
                    True, {"invalid": True})
        next_state, terms, terminated = self._transition(previous, goals)
        try:
            validate_state(next_state)
        except (TypeError, ValueError):
            return (encode_observation(previous),
                    self._weights.invalid_transition,
                    True, {"invalid": True})
        switched = previous.previous_action >= 0 and previous.previous_action != action
        reward = score_reward(terms, switched, self._weights)
        self._state = replace(next_state, previous_action=action)
        return (encode_observation(self._state), reward,
                bool(terminated or terms.invalid_transition),
                {"invalid": bool(terms.invalid_transition)})
```

`step` returns `(observation, reward, terminated, info)`. Invalid action,
infeasible goals, non-finite next state, or `invalid_transition=True` must
terminate and apply the invalid penalty. Do not implement robot physics; the
injected callback is the simulation boundary.

- [ ] **Step 5: Document the offline boundary**

`rl/README.md` must state that this is an environment contract, not a trained
policy; show the unittest command; list the 144-action mapping; and state that
later Webots transitions and model inference remain separate work. Explicitly
forbid direct real-robot use.

- [ ] **Step 6: Run GREEN**

Run `/usr/bin/python3 -m unittest -v rl.test_slot_assignment_env` and require
all tests to pass without third-party packages.

- [ ] **Step 7: Local checkpoint without commit**

Run `git diff --check` and `git status --short`.

### Task 4: Document and verify the complete local foundation

**Files:**
- Modify: `docs/test-mode-operation.md`

- [ ] **Step 1: Update operation boundaries**

Add a short offline-RL section stating:

- no trained policy or production inference runtime is installed;
- learned output is limited to one of 144 joint slot actions;
- invalid output falls back to deterministic assignment;
- wheel commands still pass through smoothing and final CBF;
- Python environment tests do not authorize powered motion.

- [ ] **Step 2: Run fresh C++ verification**

Run:

```bash
test_dir="$(mktemp -d /tmp/zooid-rl-final.XXXXXX)"
/usr/bin/clang++ -std=c++14 -Wall -Wextra -Werror -pedantic \
  tests/zooid_core_tests.cpp \
  manager/ZooidMessage.cpp manager/ZooidCoordinates.cpp \
  manager/ZooidCbfSafety.cpp manager/ZooidSpeedCodec.cpp \
  manager/ZooidTestTargets.cpp manager/ZooidPursuitRoles.cpp \
  manager/ZooidPursuitSlotPolicy.cpp manager/ZooidPursuitGeometry.cpp \
  manager/ZooidPursuitStateMachine.cpp manager/ZooidPursuitControl.cpp \
  manager/ZooidTestMode.cpp -o "$test_dir/zooid_core_tests"
"$test_dir/zooid_core_tests"
```

Expected: `zooid core tests passed`.

- [ ] **Step 3: Run sanitizer verification**

Compile the same sources with:

```text
-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer
```

Run the binary and require `zooid core tests passed` with no sanitizer report.

- [ ] **Step 4: Run Python and repository checks**

Run:

```bash
/usr/bin/python3 -m unittest -v rl.test_slot_assignment_env
git diff --check
git status --short --branch
```

Expected: Python tests pass; only the approved uncommitted design, plan, C++,
Python, project-file, test, and documentation changes are listed.

- [ ] **Step 5: Review without commit**

Request spec-compliance review, then code-quality review, then a final complete
diff review. Fix every Critical or Important issue and rerun all verification.
Leave the branch and worktree intact with uncommitted changes.
