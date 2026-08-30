# Fivefold Pursuit Speed Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Set the pursuit test to five times its original linear speeds.

**Architecture:** Adjust only phase-specific ceilings in the isolated pursuit controller, update real-code expectations, and rebuild the existing application.

**Tech Stack:** C++14, Qt 5.9.7, MinGW 5.3, qmake.

## Global Constraints

- Keep wheel saturation at `[-1000, 1000]`.
- Keep slew limiting and all safety-stop behavior unchanged.
- Keep geometry, angular control, and random-ID role assignment unchanged.

---

### Task 1: Fivefold phase speeds

**Files:**
- Modify: `tests/zooid_core_tests.cpp`
- Modify: `manager/ZooidPursuitControl.cpp`
- Modify: `docs/TEST_MODE.md`

**Interfaces:**
- Consumes: `targetEscapeCommand(...)` and `computePursuitCommands(...)`.
- Produces: updated commands through the unchanged `PursuitControlOutput`.

- [x] **Step 1: Set test expectations to fivefold speeds**

Set target expectations to `0.040`, `0.035`, and `0.025` m/s. Require a
pursuer forward wheel average greater than 30 but no greater than 50 in the
existing coordinated-controller fixture.

- [x] **Step 2: Run the core suite and observe expected speed failures**

Compile and run `tests/zooid_core_tests.exe`; the tenfold controller must fail
the new target or pursuer ceiling assertion.

- [x] **Step 3: Halve the current tenfold production ceilings**

Set target constants to `0.040`, `0.050`, `0.035`, and `0.025`, and pursuer
constants to `0.100`, `0.085`, and `0.070` in `ZooidPursuitControl.cpp`.

- [x] **Step 4: Update the operator speed documentation**

Replace the tenfold values in `docs/TEST_MODE.md` with the fivefold values.

- [x] **Step 5: Run the full core suite and rebuild the Qt executable**

Require `zooid core tests passed`, core exit code `0`, and Qt build exit code
`0`.

- [x] **Step 6: Preserve the verified non-Git workspace in place**

Do not attempt a commit because `ZooidManager` is not a Git repository.
