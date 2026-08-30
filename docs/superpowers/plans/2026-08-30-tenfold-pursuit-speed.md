# Tenfold Pursuit Speed Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Increase three-stage pursuit linear speeds to ten times their current values without weakening existing safety behavior.

**Architecture:** Change only the phase-specific linear ceilings in the isolated pursuit controller. Protect the behavior with the existing real-code core test and update the operator documentation.

**Tech Stack:** C++14, Qt 5.9.7, MinGW 5.3, qmake.

## Global Constraints

- Keep wheel commands saturated to `[-1000, 1000]`.
- Keep wheel slew limiting and all safety-stop behavior unchanged.
- Keep phase geometry and random-ID role assignment unchanged.

---

### Task 1: Tenfold phase speeds

**Files:**
- Modify: `tests/zooid_core_tests.cpp`
- Modify: `manager/ZooidPursuitControl.cpp`
- Modify: `docs/TEST_MODE.md`

**Interfaces:**
- Consumes: `targetEscapeCommand(...)` and `computePursuitCommands(...)`.
- Produces: higher wheel commands through the unchanged `PursuitControlOutput` interface.

- [x] **Step 1: Change the controller behavior test expectations**

Expect target phase ceilings of `0.100`, `0.070`, and `0.050` m/s and pursuer
phase ceilings of `0.200`, `0.170`, and `0.140` m/s.

- [x] **Step 2: Run the core test and verify RED**

Compile and run `tests/zooid_core_tests.exe`; expect `target-phase-speed failed`
or the new pursuer-speed assertion to fail against the old constants.

- [x] **Step 3: Change the production speed ceilings**

Multiply every phase-specific linear ceiling in `ZooidPursuitControl.cpp` by
ten while leaving angular speed and safety constraints unchanged.

- [x] **Step 4: Update operator documentation**

Document the new target and pursuer phase speeds in `docs/TEST_MODE.md`.

- [x] **Step 5: Verify GREEN and build the application**

Run the full core test suite, regenerate the clean qmake build, and compile the
debug executable. Expected results are `zooid core tests passed` and build exit
code `0`.

- [x] **Step 6: Preserve workspace delivery**

This folder is not a Git repository, so leave the verified files in place and
do not attempt a commit.
