# CBF Safety Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a locally verified, fail-closed CBF safety layer for the confirmed `1.460 m x 0.914 m` e-puck field, with trustworthy coordinate and receiver-state inputs.

**Architecture:** Add a Qt-free coordinate mapper and final wheel-command CBF filter. Integrate the filter after nominal command smoothing, then repair the receiver message contract so payload length and host arrival time reach the controller without LIFO rollback.

**Tech Stack:** C++14, Apple Clang core tests, existing Qt 5/qmake Windows application; no new solver or runtime dependency.

---

## Core test command

After each production source is introduced, add it to this command:

```bash
test_dir="$(mktemp -d /tmp/zooid-core-tests.XXXXXX)"
/usr/bin/clang++ -std=c++14 -Wall -Wextra -pedantic \
  tests/zooid_core_tests.cpp \
  manager/ZooidMessage.cpp \
  manager/ZooidSpeedCodec.cpp \
  manager/ZooidTestTargets.cpp \
  manager/ZooidPursuitRoles.cpp \
  manager/ZooidPursuitGeometry.cpp \
  manager/ZooidPursuitStateMachine.cpp \
  manager/ZooidPursuitControl.cpp \
  manager/ZooidTestMode.cpp \
  -o "$test_dir/zooid_core_tests"
"$test_dir/zooid_core_tests"
```

Expected output: `zooid core tests passed`, exit `0`, no warnings.

### Task 1: Canonical field and coordinate mapping

**Files:**
- Create: `manager/ZooidCoordinates.h`
- Create: `manager/ZooidCoordinates.cpp`
- Modify: `manager/ZooidPursuitTypes.h:66-74`
- Modify: `tests/zooid_core_tests.cpp`

- [ ] **Step 1: Write the failing coordinate test**

Add `testConfirmedFieldAndCoordinateMapping()` to the core test file. It must
assert all four raw endpoints, the midpoint, rejection of `rawX=62` and
`worldX=-0.001`, round-trip error no greater than one raw count, and
`PursuitWorldState` defaults equal to `1.460 x 0.914`.

Use this desired API in the test:

```cpp
#include "../manager/ZooidCoordinates.h"

ZooidWorldPoint world;
bool ok = zooidRawToWorld(63, 229, world);
uint16_t rawX = 0;
uint16_t rawY = 0;
bool roundTrip = zooidWorldToRaw(world, rawX, rawY);
```

- [ ] **Step 2: Run the core build and verify RED**

Expected failure: missing `ZooidCoordinates.h` or undefined symbols.

- [ ] **Step 3: Implement the mapper**

Create this exact public contract:

```cpp
constexpr double ZooidFieldWidth = 1.460;
constexpr double ZooidFieldHeight = 0.914;
constexpr uint16_t ZooidRawMinX = 63;
constexpr uint16_t ZooidRawMaxX = 960;
constexpr uint16_t ZooidRawMinY = 229;
constexpr uint16_t ZooidRawMaxY = 795;

struct ZooidWorldPoint { double x; double y; };

bool zooidRawToWorld(uint16_t rawX, uint16_t rawY, ZooidWorldPoint& world);
bool zooidWorldToRaw(const ZooidWorldPoint& world,
                     uint16_t& rawX, uint16_t& rawY);
```

Implementation requirements: inclusive range checks, finite world checks,
approved X-reversed/Y-positive formulas, and `std::lround` inverse mapping.
Include the header from `ZooidPursuitTypes.h` and use its field constants.

- [ ] **Step 4: Run the core test and verify GREEN**

Add `manager/ZooidCoordinates.cpp` to the command. Require exit `0`, success
text, and no warnings.

- [ ] **Step 5: Commit**

```bash
git add manager/ZooidCoordinates.h manager/ZooidCoordinates.cpp \
  manager/ZooidPursuitTypes.h tests/zooid_core_tests.cpp
git commit -m "fix: canonicalize Zooid field coordinates"
```

### Task 2: Final wheel-command CBF kernel

**Files:**
- Create: `manager/ZooidCbfSafety.h`
- Create: `manager/ZooidCbfSafety.cpp`
- Modify: `tests/zooid_core_tests.cpp`

- [ ] **Step 1: Write failing CBF tests**

Tests must cover: unchanged safe nominal commands; intervention at each field
edge; two robots closing at `0.105 m`; current distance `0.099 m` returning all
zero; age `99 ms` accepted and `100 ms` rejected; snapshot skew `51 ms`
rejected; missing command, duplicate ID, NaN, corner constraints, wheel bounds,
and input-order invariance.

Use this desired API:

```cpp
struct CbfRobotState {
    unsigned int id = 0;
    PursuitPose pose;
    uint64_t feedbackMs = 0;
};

const ZooidCbfResult result = applyZooidCbf(
    robots, nominalCommands, nowMs, ZooidCbfConfig{});
```

- [ ] **Step 2: Run the core build and verify RED**

Expected failure: missing CBF header or undefined CBF symbols.

- [ ] **Step 3: Implement the public CBF contract**

```cpp
struct ZooidCbfConfig {
    double fieldWidth = ZooidFieldWidth;
    double fieldHeight = ZooidFieldHeight;
    double safeRadius = 0.050;
    double minimumDistance = 0.100;
    double gamma = 4.0;
    double commandUnitsPerMetrePerSecond = 1000.0;
    uint64_t freshnessLimitMs = 100;
    uint64_t maximumSnapshotSkewMs = 50;
    int maximumWheelCommand = 1000;
    unsigned int maximumIterations = 256;
    double tolerance = 1e-9;
};

enum class ZooidCbfStatus {
    Safe, Intervened, InvalidInput, UnsafeState, SolverFailure
};

struct ZooidCbfResult {
    ZooidCbfStatus status = ZooidCbfStatus::InvalidInput;
    std::map<unsigned int, WheelCommand> commands;
};

ZooidCbfResult applyZooidCbf(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& nominal,
    uint64_t nowMs,
    const ZooidCbfConfig& config = {});

bool zooidCbfConstraintsSatisfied(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& commands,
    const ZooidCbfConfig& config = {});
```

Implementation algorithm:

1. Sort by ID and validate config, IDs, pose, timestamp, freshness and skew.
2. Reject centres outside the shrunken field and distances below `d_min`.
3. Optimize the wheel common mode `(left+right)/2`; preserve differential mode.
4. Add non-reverse and wheel-limit halfspaces.
5. Add four boundary CBF halfspaces per robot and one pairwise CBF halfspace
   per pair using the equations in the approved design.
6. Use deterministic Dykstra projection with 256 iterations and independent
   residual verification.
7. Reconstruct integer wheels with `std::lround` and recheck constraints.
8. Any invalid, unsafe, non-converged, or non-finite result returns exact zero
   commands for every input ID.

- [ ] **Step 4: Run the CBF tests and verify GREEN**

Add `manager/ZooidCbfSafety.cpp` to the command. Require all core tests green,
exit `0`, and no warnings.

- [ ] **Step 5: Commit**

```bash
git add manager/ZooidCbfSafety.h manager/ZooidCbfSafety.cpp \
  tests/zooid_core_tests.cpp
git commit -m "feat: add final wheel-command CBF filter"
```

### Task 3: Integrate CBF as final mission authority

**Files:**
- Modify: `manager/ZooidPursuitTypes.h:21-31`
- Modify: `manager/ZooidPursuitControl.cpp:224-241`
- Modify: `manager/ZooidTestMode.cpp:96-162`
- Modify: `ZooidManager.pro`
- Modify: `tests/zooid_core_tests.cpp`

- [ ] **Step 1: Write the failing mission safety test**

Start with four fresh robots, put one centre at `x=0.049`, call
`update(..., 1050, 1.460, 0.914, 1)`, and require:

```cpp
output.fault == PursuitFault::SafetyViolation;
output.event == "safety_stop";
mode.isRunning() == false;
```

Also assert every returned wheel command is exactly zero.

- [ ] **Step 2: Run and verify RED**

Expected failure: missing fault enum or mission continues moving.

- [ ] **Step 3: Add the single final safety call**

Add `SafetyViolation` to `PursuitFault`. Remove the old pursuer-only
`separation < 0.14` heuristic. In `ZooidTestMode::update()`, after extra robots
receive nominal zero and before returning, convert every connected/activated
robot to `CbfRobotState` and call `applyZooidCbf()`.

For `InvalidInput`, `UnsafeState`, or `SolverFailure`, call:

```cpp
return faultOutput(PursuitFault::SafetyViolation, robots);
```

For `Safe` or `Intervened`, replace `output.commands` with the filtered map.
Add both new modules to `ZooidManager.pro`.

- [ ] **Step 4: Run and verify GREEN**

Run all core tests and `git diff --check`. Require success, no warnings, and no
whitespace errors.

- [ ] **Step 5: Commit**

```bash
git add manager/ZooidPursuitTypes.h manager/ZooidPursuitControl.cpp \
  manager/ZooidTestMode.cpp ZooidManager.pro tests/zooid_core_tests.cpp
git commit -m "feat: enforce CBF before mission commands"
```

### Task 4: Preserve receiver length, FIFO order, and arrival time

**Files:**
- Modify: `manager/ZooidMessage.h`
- Modify: `manager/ZooidMessage.cpp`
- Modify: `manager/ZooidReceiver.h`
- Modify: `manager/ZooidReceiver.cpp`
- Modify: `manager/ZooidManager.cpp:3008-3066`
- Modify: `manager/ZooidManager.cpp:3138-3152`
- Modify: `tests/zooid_core_tests.cpp`

- [ ] **Step 1: Write failing message contract tests**

Tests must create an eight-byte status message, copy it, verify payload
ownership, length, arrival time and sequence, decode exact little-endian status
fields, reject lengths 7 and 9, and push messages A/B/C through a capacity-3
queue that pops A/B/C in order.

Use this desired API:

```cpp
ZooidMessage message(sender, type, bytes, length, receivedAtMs, sequence);
DecodedStatusMessage status;
bool valid = decodeStatusMessage(message, status);
ZooidMessageQueue queue(3);
queue.push(message);
ZooidMessage oldest = queue.pop();
```

- [ ] **Step 2: Run and verify RED**

Expected failure: constructor, getters, decoder, and FIFO type are absent.

- [ ] **Step 3: Implement owned messages and bounded FIFO**

Replace the raw payload pointer with `std::vector<uint8_t>`. Preserve type and
sender accessors, add exact length, `receivedAtMs`, and sequence getters. Define
`DecodedStatusMessage` with two `uint16_t`, one `int16_t`, and two `uint8_t`
fields. Decode exactly eight little-endian bytes. Implement
`ZooidMessageQueue` with `std::deque`; capacity overflow discards the oldest
message and `pop()` always returns the oldest.

- [ ] **Step 4: Run message tests and verify GREEN**

Compile with `manager/ZooidMessage.cpp`; require the complete core suite green.

- [ ] **Step 5: Integrate the contract into the Qt receiver**

Make these exact changes:

- append all positive serial reads, including 1-5 byte fragments;
- reject payload length above 32 and require `length + 5` complete bytes;
- stamp a complete valid frame with `steady_clock` and increment host sequence;
- replace vector/LIFO calls with the tested bounded FIFO;
- compare handshake bytes with exact length rather than `string(char*)`;
- decode status explicitly in `ZooidManager`, never with native `memcpy`;
- propagate `msg.getReceivedAtMs()` into `testFeedbackMs`;
- map status with `zooidRawToWorld()`;
- map outgoing positions with `zooidWorldToRaw()` and correct the negative-Y
  clamp to modify Y.

- [ ] **Step 6: Verify the available local boundary**

```bash
/usr/bin/clang++ -std=c++14 -Wall -Wextra -pedantic -fsyntax-only \
  manager/ZooidMessage.cpp manager/ZooidCoordinates.cpp manager/ZooidCbfSafety.cpp
git diff --check
```

Also run the full core suite. Require success and no warnings. Record full
Qt/Windows compilation as pending because Qt and Windows APIs are unavailable
on this Mac.

- [ ] **Step 7: Commit**

```bash
git add manager/ZooidMessage.h manager/ZooidMessage.cpp \
  manager/ZooidReceiver.h manager/ZooidReceiver.cpp manager/ZooidManager.cpp \
  tests/zooid_core_tests.cpp
git commit -m "fix: preserve fresh receiver state"
```

### Task 5: Document and verify the offline safety gate

**Files:**
- Modify: `docs/test-mode-operation.md`

- [ ] **Step 1: Document fixed limits and the hardware prohibition**

Record field size, offline `d_min`, `r_safe`, freshness and skew limits, the
CBF final-command location, the exact core command, and the prohibition on
powered movement until Windows build, protocol capture, wheel calibration,
braking, latency and staged physical tests are complete.

- [ ] **Step 2: Run fresh final verification**

Run the full core test command, Qt-free syntax check, `git diff --check`, then:

```bash
git status --short --branch
git log --oneline --decorate -8
```

- [ ] **Step 3: Commit documentation**

```bash
git add docs/test-mode-operation.md
git commit -m "docs: add CBF offline safety gate"
```

- [ ] **Step 4: Request final review**

Give the reviewer the approved design, this plan, and complete Git range. Fix
every Critical or Important finding and rerun all verification before any
completion claim.
