# Three-Stage Pursuit Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the fixed synchronized wheel sequence with a feedback-driven one-target/three-pursuer `PURSUIT -> SURROUND -> CAPTURE -> CAPTURED` hardware test using arbitrary online Zooid IDs.

**Architecture:** Pure C++14 pursuit components own role mapping, world models, geometry, state transitions, goal assignment, target behavior, and differential-drive commands. `ZooidManager` snapshots real feedback, calls the pure controller at 100 ms cadence, sends each role's command through the existing speed frame, and exposes an immutable UI snapshot. Qt and USB stay outside the deterministic core.

**Tech Stack:** C++14, Qt 5.9.7 Widgets/SerialPort, MinGW 5.3, qmake, existing ZooidManager receiver protocol.

## Global Constraints

- Exactly four selected robots: lowest fresh real ID is target; next three are pursuers; additional robots receive zero speed.
- No ROS, Python, Webots, reinforcement learning, Voronoi control, or obstacle planner at runtime.
- Geometry uses the hardware-table radii `0.31 m`, `0.36 m`, `0.24 m`, `0.17 m`, and capture-slot radius `0.1224 m`.
- Pursuer linear limits are `0.020`, `0.017`, and `0.014 m/s`; target limits are `0.010`, `0.007`, and `0.005 m/s` by phase.
- Signed wheel commands are slew-limited and saturated to `[-1000, 1000]` before the existing `+2007` speed-frame encoding.
- Missing/non-finite/stale participant feedback, receiver write failure, manual stop, and application shutdown cause an immediate three-cycle zero-speed burst.
- `CAPTURED` stops all four robots; post-capture orbit is excluded.
- New behavior follows strict RED -> verify failure -> GREEN -> verify pass cycles.
- The source directory is not a Git repository; replace commit checkpoints with a changed-file/status check.

---

### Task 1: Role Mapping and Feedback Models

**Files:**
- Create: `manager/ZooidPursuitTypes.h`
- Create: `manager/ZooidPursuitRoles.h`
- Create: `manager/ZooidPursuitRoles.cpp`
- Modify: `tests/zooid_core_tests.cpp`

**Interfaces:**
- Produces `PursuitPose`, `PursuitRobotState`, `PursuitWorldState`, `PursuitRoleMap`, `PursuitPhase`, `PursuitFault`, and `PursuitStatusSnapshot`.
- Produces `bool assignPursuitRoles(const std::vector<unsigned int>& freshIds, PursuitRoleMap& out)` and `std::vector<unsigned int> PursuitRoleMap::participantIds() const`.

- [ ] **Step 1: Write failing role tests**

Add literal assertions that `{17, 4, 29, 8, 2}` maps target `2` and pursuers `{4,8,17}`, duplicates are normalized, fewer than four IDs fail without altering the output, additional ID `29` is excluded, and a copied role map does not change when later input changes.

- [ ] **Step 2: Compile and verify RED**

```powershell
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\g++.exe' -std=c++14 -Wall -Wextra -pedantic tests\zooid_core_tests.cpp manager\ZooidPursuitRoles.cpp -o tests\zooid_core_tests.exe
```

Expected: compilation fails because the new headers/types/functions do not exist.

- [ ] **Step 3: Implement minimal role and model code**

Use fixed three-element pursuer storage and explicit validity:

```cpp
struct PursuitRoleMap {
    unsigned int targetId = 0;
    std::array<unsigned int, 3> pursuerIds{{0, 0, 0}};
    bool valid = false;
    std::vector<unsigned int> participantIds() const;
};

bool assignPursuitRoles(const std::vector<unsigned int>& freshIds,
                        PursuitRoleMap& out);
```

Sort/unique a copy, require four entries, select only the first four, and assign `out` only after validation succeeds.

- [ ] **Step 4: Recompile and verify GREEN**

Run the executable and expect `zooid core tests passed`.

- [ ] **Step 5: Record checkpoint**

List the three new files and the updated test file; do not modify generated executables as source artifacts.

### Task 2: Pursuit Geometry and Persistent Slot Assignment

**Files:**
- Create: `manager/ZooidPursuitGeometry.h`
- Create: `manager/ZooidPursuitGeometry.cpp`
- Modify: `tests/zooid_core_tests.cpp`

**Interfaces:**
- Consumes `PursuitPose`, `PursuitRobotState`, and `PursuitWorldState`.
- Produces `distanceBetween`, `normalizeAngle`, `circularAngularGaps`, `makeTriangularRing`, `ringInsideBounds`, `captureGeometrySatisfied`, and `surroundGeometrySatisfied`.
- Produces `PursuitSlotAssigner::assign(world, phase, radius, nonExpanding)` and `clear()`.

- [ ] **Step 1: Write failing geometry tests**

Use hand-derived fixtures around target `(0.50, 0.50)` to verify radius `0.24`, bearings `0`, `2pi/3`, `4pi/3`, headings toward the target, literal angular gaps near `2.094395`, rectangular boundary rejection, and capture success/failure at `0.17 m`.

- [ ] **Step 2: Compile and verify RED**

Expected: missing geometry declarations.

- [ ] **Step 3: Implement geometry primitives**

Implement finite checks and field bounds without Qt. Sample 24 ring headings, reject any slot outside `[margin, width-margin] x [margin, height-margin]`, enumerate all six pursuer-slot permutations, and minimize total squared distance.

- [ ] **Step 4: Add failing persistence tests**

Verify that a small target motion retains the same role-to-bearing assignment, a phase change permits reassignment, and `nonExpanding=true` never returns a goal farther from the target than that pursuer's current radius.

- [ ] **Step 5: Implement `PursuitSlotAssigner` and verify GREEN**

Store remembered bearings by phase; clear on reset or infeasible recovery. Run all core tests and expect success.

### Task 3: Fresh-Observation State Machine and Fallbacks

**Files:**
- Create: `manager/ZooidPursuitStateMachine.h`
- Create: `manager/ZooidPursuitStateMachine.cpp`
- Modify: `tests/zooid_core_tests.cpp`

**Interfaces:**
- Consumes `PursuitWorldState` and geometry predicates.
- Produces `PursuitPhaseResult PursuitStateMachine::update(const PursuitWorldState&)`, `start()`, `stop()`, `reset()`, and `phase()`.

- [ ] **Step 1: Write failing transition tests**

Construct fresh literal worlds for:

- IDLE remains stopped until `start()`;
- 35 PURSUIT ticks plus three ready observations enter SURROUND;
- 35 SURROUND ticks plus three geometrically ready observations enter CAPTURE;
- 20 contained observations enter CAPTURED;
- duplicate sequence/stamp observations do not advance counters.

- [ ] **Step 2: Compile/run and verify RED**

Expected: missing state-machine API.

- [ ] **Step 3: Implement minimal forward transitions**

Use a configuration struct initialized to hardware-table literals. Reset phase-local counters and slot evidence on every transition. Do not include strategy-advice or learned-policy gates.

- [ ] **Step 4: Write failing fallback tests**

Verify eight broken SURROUND samples return to PURSUIT, six post-acquisition broken CAPTURE samples return to SURROUND, a far CAPTURE escape returns to PURSUIT, and infeasible rings cannot advance.

- [ ] **Step 5: Implement fallbacks and verify GREEN**

Port only deterministic freshness, readiness, acquisition, break, and timeout logic from `strategy/state_machine.py`; keep diagnostics in `PursuitPhaseResult` for UI use.

### Task 4: Target/Pursuer Motion and Wheel Conversion

**Files:**
- Create: `manager/ZooidPursuitControl.h`
- Create: `manager/ZooidPursuitControl.cpp`
- Modify: `tests/zooid_core_tests.cpp`

**Interfaces:**
- Consumes phase, world, role map, field dimensions, and assigned goals.
- Produces `PursuitTwist driveToward(...)`, `PursuitTwist targetEscapeCommand(...)`, `WheelCommand differentialDrive(...)`, `WheelCommandSmoother`, and `PursuitControlOutput computePursuitCommands(...)` keyed by real ID.

- [ ] **Step 1: Write failing point-controller and heading tests**

Verify receiver headings `0`, `90`, `180`, `270` degrees convert to mathematical yaw `pi/2`, `0`, `-pi/2`, and `pi`; a robot facing its goal drives forward; a robot facing away rotates before translating; non-finite input produces a fault and zero commands.

- [ ] **Step 2: Verify RED, then implement point control**

Use `linear = min(distance, phaseMax) * max(0, cos(error))^2` and `angular = clamp(2*error, -1.8, 1.8)`, with translation zero for large heading error.

- [ ] **Step 3: Write failing wheel tests**

Use literal checks: `(v=0.020, omega=0)` gives equal positive wheels; positive omega makes right greater than left; negative omega reverses that ordering; saturation never exceeds 1000; one update cannot exceed configured wheel-unit slew.

- [ ] **Step 4: Implement differential drive and smoothing**

Use wheel base `0.05 m`, named speed-unit scale, proportional saturation, and per-real-ID previous commands. Preserve signed left/right semantics expected by `encodeWheelSpeeds`.

- [ ] **Step 5: Write failing target and pairwise-safety tests**

Verify the target selects an inward feasible escape direction near each field edge, stops in IDLE/CAPTURED, observes the three phase speed caps, and pursuer commands cannot reduce an already-close pair distance.

- [ ] **Step 6: Implement target escape, boundary guard, pairwise guard, and verify GREEN**

Use the aggregate direction away from pursuers, deterministic candidate rotations when blocked, a `0.057 m` field margin, minimum participant separation `0.14 m`, and zero as the no-safe-command fallback.

### Task 5: Mission Orchestrator and Safety Contract

**Files:**
- Replace: `manager/ZooidTestMode.h`
- Replace: `manager/ZooidTestMode.cpp`
- Replace: `manager/ZooidTestTargets.h`
- Replace: `manager/ZooidTestTargets.cpp`
- Modify: `tests/zooid_core_tests.cpp`

**Interfaces:**
- Consumes role assignment, state machine, slot assignment, and control pipeline.
- Produces `bool ZooidTestMode::start(const std::vector<PursuitRobotState>&, uint64_t)`, `PursuitControlOutput update(...)`, `stop()`, `roleMap()`, and `statusSnapshot()`.

- [ ] **Step 1: Replace fixed-sequence tests with failing mission tests**

Remove expectations for timed forward/turn/reverse commands. Add tests for arbitrary IDs, exact four-role output, zero commands for extra IDs, frozen mapping, stage status text data, CAPTURED zero output, restart reset, and failure to start with three robots.

- [ ] **Step 2: Compile/run and verify RED**

Expected: old `start(uint64_t)` and single shared wheel command cannot satisfy the new API.

- [ ] **Step 3: Implement the mission orchestrator**

Compose the pure components, estimate bounded target velocity from fresh samples, produce one command per real participant ID, and return explicit `PursuitFault` values for missing, stale, and invalid feedback.

- [ ] **Step 4: Add failing timeout/loss/idempotence tests**

Verify 499 ms remains valid, 500 ms faults, one missing selected ID faults even if another robot joins, repeated stop remains zero, and a new start rebuilds roles from a new sorted snapshot.

- [ ] **Step 5: Implement safety state and verify GREEN**

Keep the frozen participant list available after a fault so the manager can send zero to both still-online and just-lost IDs.

### Task 6: ZooidManager and UI Integration

**Files:**
- Modify: `manager/ZooidManager.h`
- Modify: `manager/ZooidManager.cpp`
- Modify: `homePage.h`
- Modify: `homePage.cpp`
- Modify: `ZooidManager.pro`

**Interfaces:**
- Consumes the pure orchestrator.
- Produces `PursuitStatusSnapshot ZooidManager::getTestModeSnapshot() const` for the UI.

- [ ] **Step 1: Add new sources to qmake and verify the application build is RED**

Add all new `.cpp` and `.h` files to `ZooidManager.pro`, update manager declarations to the new API, and run qmake/make. Expected: build fails at old fixed-sequence integration sites.

- [ ] **Step 2: Replace manager snapshots with full feedback snapshots**

Under `valuesMutex`, copy real ID, position, orientation, connection/activation state, and a monotonic feedback timestamp. Do not hold `valuesMutex` while calculating controls or writing USB data.

- [ ] **Step 3: Integrate per-ID command dispatch**

At StartPending, require four fresh robots and build roles. While Running, update the orchestrator, send target and pursuer commands by real ID, and explicitly send zero to every extra active robot. Any fault calls the existing queue-clear and three-cycle safe-stop path.

- [ ] **Step 4: Expose stable UI snapshot**

Return one mutex-protected copy containing status, phase, target ID, three pursuer IDs, distances, capture progress, lost IDs, and latest event/fault. Avoid multiple getters that could observe different ticks.

- [ ] **Step 5: Update the two-button UI**

Keep only Start Test and Stop. Render Chinese text such as:

```text
阶段: SURROUND
目标 ID: 2
追捕 ID: 4, 8, 17
距离: 0.25, 0.24, 0.26 m
捕获进度: 0%
```

Disable Start during StartPending/Running/safe-stop burst; keep Stop enabled throughout active stages.

- [ ] **Step 6: Build and verify GREEN**

```powershell
New-Item -ItemType Directory -Force -Path '.build-three-stage' | Out-Null
Push-Location '.build-three-stage'
& 'D:\Qt5.9.7\5.9.7\mingw53_32\bin\qmake.exe' '..\ZooidManager.pro'
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\mingw32-make.exe' -j2 debug
Pop-Location
```

Expected: exit code 0 and `.build-three-stage/debug/ZooidManager.exe` exists.

### Task 7: Full Regression and Delivery Record

**Files:**
- Modify: `docs/TEST_MODE.md`
- Modify: `docs/superpowers/plans/2026-08-30-three-stage-pursuit-test.md`

- [ ] **Step 1: Run the complete pure-core suite**

```powershell
& 'D:\Qt5.9.7\Tools\mingw530_32\bin\g++.exe' -std=c++14 -Wall -Wextra -pedantic tests\zooid_core_tests.cpp manager\ZooidSpeedCodec.cpp manager\ZooidPursuitRoles.cpp manager\ZooidPursuitGeometry.cpp manager\ZooidPursuitStateMachine.cpp manager\ZooidPursuitControl.cpp manager\ZooidTestMode.cpp manager\ZooidTestTargets.cpp -o tests\zooid_core_tests.exe
& '.\tests\zooid_core_tests.exe'
```

Expected: compilation has no warnings and output is `zooid core tests passed`.

- [ ] **Step 2: Perform a clean Qt build**

Delete only the validated workspace-local `.build-three-stage` directory, recreate it, run qmake and `mingw32-make -j2 debug`, and verify exit code 0.

- [ ] **Step 3: Update operator documentation**

Document automatic random-ID mapping, the three stages, displayed diagnostics, conservative speed defaults, receiver requirement, Stop Test behavior, and the rule that real motion still requires a restrained clear-table trial.

- [ ] **Step 4: Self-review against the design specification**

Check every requirement in `docs/superpowers/specs/2026-08-30-three-stage-pursuit-test-design.md`, scan for fixed `0/1/2/3` role assumptions, ensure extra robots are zeroed, and confirm CAPTURED cannot emit motion.

- [ ] **Step 5: Record final artifact and limitations**

Report the absolute executable path, exact test/build commands and results, changed source files, and the fact that physical wheel calibration and live capture performance cannot be proven without running the hardware.
