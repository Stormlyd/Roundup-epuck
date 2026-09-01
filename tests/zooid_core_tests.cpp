#include "../manager/ZooidTestMode.h"
#include "../manager/ZooidSpeedCodec.h"
#include "../manager/ZooidTestTargets.h"
#include "../manager/ZooidPursuitRoles.h"
#include "../manager/ZooidPursuitGeometry.h"
#include "../manager/ZooidPursuitStateMachine.h"
#include "../manager/ZooidPursuitControl.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

static int failures = 0;

static void testSpeedCodec()
{
    const EncodedWheelSpeeds forward = encodeWheelSpeeds(200, 200);
    if (forward.positionX != 2207 || forward.positionY != 2207) ++failures;

    const EncodedWheelSpeeds turn = encodeWheelSpeeds(-150, 150);
    if (turn.positionX != 1857 || turn.positionY != 2157) ++failures;

    const EncodedWheelSpeeds stop = encodeWheelSpeeds(0, 0);
    if (stop.positionX != 2007 || stop.positionY != 2007) ++failures;

    const EncodedWheelSpeeds clamped = encodeWheelSpeeds(1200, -1300);
    if (clamped.left != 1000 || clamped.right != -1000 ||
        clamped.positionX != 3007 || clamped.positionY != 1007) ++failures;
}

static void expectIds(const char* name,
                      const std::vector<unsigned int>& actual,
                      const std::vector<unsigned int>& expected)
{
    if (actual != expected) {
        std::cerr << name << " failed\n";
        ++failures;
    }
}

static void testTargetSnapshot()
{
    ZooidTestTargets targets;
    targets.startSnapshot({12, 1, 12, 5});
    expectIds("normalized-snapshot", targets.activeIds(), {1, 5, 12});

    targets.retainActive({1, 5, 9, 12, 20});
    expectIds("late-joiners-excluded", targets.activeIds(), {1, 5, 12});
    expectIds("no-lost-targets", targets.lostIds(), {});

    targets.retainActive({1, 9, 20});
    expectIds("disconnected-targets-removed", targets.activeIds(), {1});
    expectIds("lost-targets-recorded", targets.lostIds(), {5, 12});

    targets.retainActive({1, 5, 12});
    expectIds("lost-targets-do-not-rejoin", targets.activeIds(), {1});
    targets.clear();
    if (!targets.empty() || !targets.lostIds().empty()) ++failures;
}

static void testRandomHardwareIdsAssignFourLogicalRoles()
{
    PursuitRoleMap roles;
    if (!assignPursuitRoles({17, 4, 29, 8, 2, 8}, roles)) {
        std::cerr << "random-role-assignment failed\n";
        ++failures;
        return;
    }
    if (!roles.valid || roles.targetId != 2 ||
        roles.pursuerIds != std::array<unsigned int, 3>{{4, 8, 17}}) {
        std::cerr << "sorted-logical-roles failed\n";
        ++failures;
    }
    expectIds("only-four-participants", roles.participantIds(), {2, 4, 8, 17});
}

static void testRoleAssignmentRejectsTooFewWithoutMutation()
{
    PursuitRoleMap roles;
    roles.targetId = 99;
    roles.pursuerIds = {{91, 92, 93}};
    roles.valid = true;
    if (assignPursuitRoles({7, 3, 5}, roles)) {
        std::cerr << "too-few-role-assignment accepted\n";
        ++failures;
    }
    if (roles.targetId != 99 || roles.pursuerIds != std::array<unsigned int, 3>{{91, 92, 93}} || !roles.valid) {
        std::cerr << "failed-assignment-mutated-output failed\n";
        ++failures;
    }
}

static bool near(double actual, double expected, double tolerance = 1e-6)
{
    return std::abs(actual - expected) <= tolerance;
}

static void testTriangularRingUsesHandCheckedGeometry()
{
    const PursuitPose target{0.50, 0.50, 0.0};
    const auto ring = makeTriangularRing(target, 0.24, 0.0);
    if (!near(ring[0].x, 0.74) || !near(ring[0].y, 0.50) ||
        !near(ring[1].x, 0.38) || !near(ring[1].y, 0.7078460969) ||
        !near(ring[2].x, 0.38) || !near(ring[2].y, 0.2921539031)) {
        std::cerr << "triangular-ring-coordinates failed\n";
        ++failures;
    }
    const auto gaps = circularAngularGaps(target, ring);
    for (double gap : gaps) {
        if (!near(gap, 2.0943951024)) {
            std::cerr << "triangular-ring-gaps failed\n";
            ++failures;
            break;
        }
    }
    if (!ringInsideBounds(ring, 1.90, 1.00, 0.057) ||
        ringInsideBounds(makeTriangularRing(PursuitPose{1.70, 0.50, 0.0}, 0.24, 0.0),
                         1.90, 1.00, 0.057)) {
        std::cerr << "ring-boundary-check failed\n";
        ++failures;
    }
}

static void testCaptureGeometryChecksRadiusAndAngularContainment()
{
    const PursuitPose target{0.50, 0.50, 0.0};
    const auto contained = makeTriangularRing(target, 0.16, 0.0);
    if (!captureGeometrySatisfied(target, contained, 0.17, 1.65, 2.60)) {
        std::cerr << "contained-capture rejected\n";
        ++failures;
    }
    auto tooFar = contained;
    tooFar[0].x = 0.68;
    if (captureGeometrySatisfied(target, tooFar, 0.17, 1.65, 2.60)) {
        std::cerr << "far-capture accepted\n";
        ++failures;
    }
}

static PursuitWorldState makeGeometryWorld()
{
    PursuitWorldState world;
    world.target.pose = {0.80, 0.50, 0.0};
    world.pursuers[0].pose = {0.45, 0.50, 0.0};
    world.pursuers[1].pose = {0.90, 0.82, 0.0};
    world.pursuers[2].pose = {0.90, 0.18, 0.0};
    world.fieldWidth = 1.90;
    world.fieldHeight = 1.00;
    return world;
}

static void testSlotAssignmentsPersistAndNeverExpand()
{
    PursuitSlotAssigner assigner;
    PursuitWorldState first = makeGeometryWorld();
    std::array<PursuitPose, 3> firstGoals;
    if (!assigner.assign(first, PursuitPhase::Pursuit, 0.31, true, firstGoals)) {
        std::cerr << "initial-slot-assignment failed\n";
        ++failures;
        return;
    }
    std::array<double, 3> bearings;
    for (std::size_t i = 0; i < 3; ++i) {
        bearings[i] = std::atan2(firstGoals[i].y - first.target.pose.y,
                                 firstGoals[i].x - first.target.pose.x);
        const double currentRadius = distanceBetween(first.target.pose, first.pursuers[i].pose);
        if (distanceBetween(first.target.pose, firstGoals[i]) > currentRadius + 1e-9) {
            std::cerr << "non-expanding-slot expanded radius\n";
            ++failures;
        }
    }

    PursuitWorldState moved = first;
    moved.target.pose.x += 0.01;
    std::array<PursuitPose, 3> movedGoals;
    if (!assigner.assign(moved, PursuitPhase::Pursuit, 0.31, true, movedGoals)) {
        std::cerr << "persistent-slot-assignment failed\n";
        ++failures;
        return;
    }
    for (std::size_t i = 0; i < 3; ++i) {
        const double movedBearing = std::atan2(movedGoals[i].y - moved.target.pose.y,
                                              movedGoals[i].x - moved.target.pose.x);
        if (!near(normalizeAngle(movedBearing - bearings[i]), 0.0)) {
            std::cerr << "slot-bearing-swapped failed\n";
            ++failures;
        }
    }
}

static void testSurroundSlotsRestoreNominalRadius()
{
    PursuitWorldState world = makeGeometryWorld();
    const auto innerRing = makeTriangularRing(world.target.pose, 0.15, 0.0);
    for (std::size_t i = 0; i < 3; ++i)
        world.pursuers[i].pose = innerRing[i];

    PursuitSlotAssigner assigner;
    std::array<PursuitPose, 3> goals;
    if (!assigner.assign(world, PursuitPhase::Surround,
                         PursuitProfile::SurroundRadius, false, goals)) {
        std::cerr << "surround-slot-assignment failed\n";
        ++failures;
        return;
    }
    for (const PursuitPose& goal : goals) {
        if (!near(distanceBetween(world.target.pose, goal),
                  PursuitProfile::SurroundRadius)) {
            std::cerr << "surround-slot-did-not-expand failed\n";
            ++failures;
            break;
        }
    }
}

static PursuitWorldState makeStateWorld(double radius, uint64_t sequence)
{
    PursuitWorldState world;
    world.target.pose = {0.80, 0.50, 0.0};
    const auto ring = makeTriangularRing(world.target.pose, radius, 0.0);
    for (std::size_t i = 0; i < 3; ++i)
        world.pursuers[i].pose = ring[i];
    world.fieldWidth = 1.90;
    world.fieldHeight = 1.00;
    world.sequence = sequence;
    world.stampMs = sequence * 100;
    return world;
}

static void testStateMachineCompletesThreeFreshObservationStages()
{
    PursuitStateMachine machine;
    if (machine.phase() != PursuitPhase::Idle || !machine.start() ||
        machine.phase() != PursuitPhase::Pursuit) {
        std::cerr << "state-machine-start failed\n";
        ++failures;
        return;
    }

    uint64_t sequence = 1;
    PursuitPhaseResult result;
    for (int i = 0; i < 37; ++i)
        result = machine.update(makeStateWorld(0.31, sequence++));
    if (result.phase != PursuitPhase::Surround || result.event != "surround_started") {
        std::cerr << "pursuit-to-surround failed\n";
        ++failures;
    }
    for (int i = 0; i < 37; ++i)
        result = machine.update(makeStateWorld(0.24, sequence++));
    if (result.phase != PursuitPhase::Capture || result.event != "capture_started") {
        std::cerr << "surround-to-capture failed\n";
        ++failures;
    }
    for (int i = 0; i < 20; ++i)
        result = machine.update(makeStateWorld(0.16, sequence++));
    if (result.phase != PursuitPhase::Captured || result.event != "captured" ||
        !near(result.captureProgress, 1.0)) {
        std::cerr << "capture-to-captured failed\n";
        ++failures;
    }
}

static void testDuplicateObservationDoesNotAdvanceDwellEvidence()
{
    PursuitStateMachine machine;
    machine.start();
    PursuitWorldState world;
    for (uint64_t sequence = 1; sequence <= 36; ++sequence)
        world = makeStateWorld(0.31, sequence), machine.update(world);
    for (int i = 0; i < 20; ++i)
        machine.update(world);
    if (machine.phase() != PursuitPhase::Pursuit) {
        std::cerr << "duplicate-observation-advanced-phase failed\n";
        ++failures;
    }
    if (machine.update(makeStateWorld(0.31, 37)).phase != PursuitPhase::Surround) {
        std::cerr << "fresh-observation-did-not-advance failed\n";
        ++failures;
    }
}

static void testBrokenSurroundFallsBackToPursuit()
{
    PursuitStateMachine machine;
    machine.start();
    uint64_t sequence = 1;
    for (int i = 0; i < 37; ++i)
        machine.update(makeStateWorld(0.31, sequence++));
    PursuitPhaseResult result;
    for (int i = 0; i < 8; ++i)
        result = machine.update(makeStateWorld(0.50, sequence++));
    if (result.phase != PursuitPhase::Pursuit ||
        result.event != "surround_broken_reacquire") {
        std::cerr << "surround-fallback failed\n";
        ++failures;
    }
}

static void testBrokenAcquiredCaptureFallsBackByDistance()
{
    PursuitStateMachine machine;
    machine.start();
    uint64_t sequence = 1;
    for (int i = 0; i < 37; ++i)
        machine.update(makeStateWorld(0.31, sequence++));
    for (int i = 0; i < 37; ++i)
        machine.update(makeStateWorld(0.24, sequence++));
    machine.update(makeStateWorld(0.16, sequence++));

    PursuitPhaseResult result;
    for (int i = 0; i < 6; ++i)
        result = machine.update(makeStateWorld(0.30, sequence++));
    if (result.phase != PursuitPhase::Surround ||
        result.event != "capture_broken_reform") {
        std::cerr << "capture-reform-fallback failed\n";
        ++failures;
    }
}

static void testCaptureAcquisitionEscapesBackToPursuit()
{
    PursuitStateMachine machine;
    machine.start();
    uint64_t sequence = 1;
    for (int i = 0; i < 37; ++i)
        machine.update(makeStateWorld(0.31, sequence++));
    for (int i = 0; i < 37; ++i)
        machine.update(makeStateWorld(0.24, sequence++));
    PursuitPhaseResult result;
    for (int i = 0; i < 8; ++i)
        result = machine.update(makeStateWorld(0.50, sequence++));
    if (result.phase != PursuitPhase::Pursuit ||
        result.event != "capture_broken_reacquire") {
        std::cerr << "capture-acquisition-fallback failed\n";
        ++failures;
    }
}

static void testReceiverHeadingAndPointController()
{
    const double pi = 3.14159265358979323846;
    if (!near(receiverHeadingToYaw(0.0), pi / 2.0) ||
        !near(receiverHeadingToYaw(90.0), 0.0) ||
        !near(receiverHeadingToYaw(180.0), -pi / 2.0) ||
        !near(std::abs(receiverHeadingToYaw(270.0)), pi)) {
        std::cerr << "receiver-heading-conversion failed\n";
        ++failures;
    }
    const PursuitTwist forward = driveToward({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.02);
    if (!near(forward.linear, 0.02) || !near(forward.angular, 0.0)) {
        std::cerr << "point-controller-forward failed\n";
        ++failures;
    }
    const PursuitTwist turn = driveToward({0.0, 0.0, pi}, {1.0, 0.0, 0.0}, 0.02);
    if (!near(turn.linear, 0.0) || std::abs(turn.angular) < 1.79) {
        std::cerr << "point-controller-turn-first failed\n";
        ++failures;
    }
}

static void testDifferentialDriveSignsLimitsAndSlew()
{
    const WheelCommand straight = differentialDrive({0.020, 0.0});
    if (straight.left != 155 || straight.right != 155) {
        std::cerr << "differential-straight failed\n";
        ++failures;
    }
    const WheelCommand leftTurn = differentialDrive({0.020, 0.4});
    if (leftTurn.left != 73 || leftTurn.right != 238) {
        std::cerr << "differential-positive-turn failed\n";
        ++failures;
    }
    const WheelCommand calibratedCruise = differentialDrive({0.100, 0.0});
    const WheelCommand calibratedSpin = differentialDrive({0.0, 1.0});
    if (calibratedCruise.left != 776 || calibratedCruise.right != 776 ||
        calibratedSpin.left != -206 || calibratedSpin.right != 206) {
        std::cerr << "epuck2-drive-calibration failed\n";
        ++failures;
    }
    const WheelCommand saturated = differentialDrive({2.0, 10.0});
    if (std::abs(saturated.left) > 1000 || std::abs(saturated.right) > 1000) {
        std::cerr << "differential-saturation failed\n";
        ++failures;
    }
    WheelCommandSmoother smoother(40);
    const WheelCommand first = smoother.smooth(17, {200, -200});
    const WheelCommand second = smoother.smooth(17, {200, -200});
    if (first.left != 40 || first.right != -40 ||
        second.left != 80 || second.right != -80) {
        std::cerr << "wheel-command-slew failed\n";
        ++failures;
    }
}

static PursuitWorldState makeControlWorld()
{
    PursuitWorldState world;
    world.target = {2, {0.80, 0.50, 0.0}, 1000, true, true, 0.0, 0.0};
    world.pursuers[0] = {4, {0.48, 0.50, 0.0}, 1000, true, true, 0.0, 0.0};
    world.pursuers[1] = {8, {0.64, 0.78, -1.0}, 1000, true, true, 0.0, 0.0};
    world.pursuers[2] = {17, {0.64, 0.22, 1.0}, 1000, true, true, 0.0, 0.0};
    world.fieldWidth = 1.90;
    world.fieldHeight = 1.00;
    world.stampMs = 1000;
    world.sequence = 1;
    return world;
}

static void testTargetEscapeRespectsPhaseSpeedAndTerminalStop()
{
    const PursuitWorldState world = makeControlWorld();
    const PursuitTwist pursuit = targetEscapeCommand(world, PursuitPhase::Pursuit);
    const PursuitTwist surround = targetEscapeCommand(world, PursuitPhase::Surround);
    const PursuitTwist capture = targetEscapeCommand(world, PursuitPhase::Capture);
    const PursuitTwist captured = targetEscapeCommand(world, PursuitPhase::Captured);
    if (!near(pursuit.linear, 0.040) ||
        !near(surround.linear, 0.035) ||
        !near(capture.linear, 0.025) ||
        !near(captured.linear, 0.0) || !near(captured.angular, 0.0)) {
        std::cerr << "target-phase-speed failed\n";
        ++failures;
    }
}

static void testCapturePressureSlowsTargetAndEdgePursuitRemainsFeasible()
{
    PursuitWorldState pressured = makeControlWorld();
    const auto closeRing = makeTriangularRing(pressured.target.pose, 0.15, 0.0);
    for (std::size_t i = 0; i < 3; ++i) pressured.pursuers[i].pose = closeRing[i];
    const PursuitTwist capture = targetEscapeCommand(pressured, PursuitPhase::Capture);
    if (capture.linear > 0.0063) {
        std::cerr << "capture-pressure-did-not-slow-target failed\n";
        ++failures;
    }

    PursuitWorldState edge = makeControlWorld();
    edge.target.pose = {0.08, 0.50, 0.0};
    edge.pursuers[0].pose = {0.35, 0.50, 3.14159265358979323846};
    edge.pursuers[1].pose = {0.30, 0.70, -2.4};
    edge.pursuers[2].pose = {0.30, 0.30, 2.4};
    PursuitRoleMap roles;
    assignPursuitRoles({2, 4, 8, 17}, roles);
    PursuitSlotAssigner slots;
    WheelCommandSmoother smoother(1000);
    const PursuitControlOutput output = computePursuitCommands(
        edge, roles, PursuitPhase::Pursuit, slots, smoother);
    if (output.fault != PursuitFault::None || output.commands.size() != 4) {
        std::cerr << "edge-pursuit-unnecessarily-faulted failed\n";
        ++failures;
    }
    const PursuitTwist recovery = targetEscapeCommand(edge, PursuitPhase::Pursuit);
    if (triangularRingFeasible(edge.target.pose,
                              PursuitProfile::SurroundRadius,
                              edge.fieldWidth, edge.fieldHeight) ||
        recovery.linear < PursuitProfile::TargetPursuitSpeed ||
        !near(recovery.angular, 0.0)) {
        std::cerr << "edge-target-did-not-return-inward failed\n";
        ++failures;
    }
    PursuitWorldState blockedEdge = edge;
    blockedEdge.pursuers[0].pose = {0.18, 0.50, 3.14159265358979323846};
    const PursuitTwist blockedRecovery = targetEscapeCommand(
        blockedEdge, PursuitPhase::Pursuit);
    if (!near(blockedRecovery.linear, 0.0)) {
        std::cerr << "blocked-edge-target-did-not-hold failed\n";
        ++failures;
    }
}

static void testFieldBoundaryBrakePreservesTurning()
{
    const double pi = 3.14159265358979323846;
    const double width = 1.46;
    const double height = 0.914;
    const double low = PursuitProfile::BoundaryMargin + 0.010;
    const double highX = width - PursuitProfile::BoundaryMargin - 0.010;
    const double highY = height - PursuitProfile::BoundaryMargin - 0.010;
    const PursuitTwist left = brakeAtFieldBoundary(
        {low, 0.50, pi}, width, height, {0.10, 0.70});
    const PursuitTwist right = brakeAtFieldBoundary(
        {highX, 0.50, 0.0}, width, height, {0.10, 0.70});
    const PursuitTwist bottom = brakeAtFieldBoundary(
        {0.73, low, -pi / 2.0}, width, height, {0.10, 0.70});
    const PursuitTwist top = brakeAtFieldBoundary(
        {0.73, highY, pi / 2.0}, width, height, {0.10, 0.70});
    const PursuitTwist inward = brakeAtFieldBoundary(
        {low, 0.50, 0.0}, width, height, {0.10, 0.70});
    const double expectedScale = 0.010 / PursuitProfile::BoundaryBrakingBand;
    if (!near(left.linear, 0.10 * expectedScale) ||
        !near(right.linear, 0.10 * expectedScale) ||
        !near(bottom.linear, 0.10 * expectedScale) ||
        !near(top.linear, 0.10 * expectedScale) ||
        !near(left.angular, 0.70) || !near(right.angular, 0.70) ||
        !near(bottom.angular, 0.70) || !near(top.angular, 0.70) ||
        !near(inward.linear, 0.10) || !near(inward.angular, 0.70)) {
        std::cerr << "field-boundary-brake failed\n";
        ++failures;
    }
}

static void testCoordinatedControllerUsesRealIdsAndStopsWhenCaptured()
{
    const PursuitWorldState world = makeControlWorld();
    PursuitRoleMap roles;
    assignPursuitRoles({17, 8, 4, 2}, roles);
    PursuitSlotAssigner slots;
    WheelCommandSmoother smoother(1000);
    const PursuitControlOutput running = computePursuitCommands(
        world, roles, PursuitPhase::Pursuit, slots, smoother);
    if (running.fault != PursuitFault::None || running.commands.size() != 4 ||
        running.commands.count(2) != 1 || running.commands.count(4) != 1 ||
        running.commands.count(8) != 1 || running.commands.count(17) != 1) {
        std::cerr << "real-id-command-map failed\n";
        ++failures;
    }
    bool hasCalibratedPursuer = false;
    bool commandExceededCodecLimit = false;
    for (unsigned int id : roles.pursuerIds) {
        const WheelCommand command = running.commands.at(id);
        hasCalibratedPursuer = hasCalibratedPursuer ||
            std::max(std::abs(command.left), std::abs(command.right)) > 200;
        commandExceededCodecLimit = commandExceededCodecLimit ||
            std::abs(command.left) > 1000 || std::abs(command.right) > 1000;
    }
    if (!hasCalibratedPursuer) {
        std::cerr << "pursuer-hardware-calibration-not-applied failed\n";
        ++failures;
    }
    if (commandExceededCodecLimit) {
        std::cerr << "pursuer-command-limit failed\n";
        ++failures;
    }
    smoother.clear();
    const PursuitControlOutput stopped = computePursuitCommands(
        world, roles, PursuitPhase::Captured, slots, smoother);
    for (const auto& entry : stopped.commands) {
        if (entry.second.left != 0 || entry.second.right != 0) {
            std::cerr << "captured-command-not-zero failed\n";
            ++failures;
            break;
        }
    }
}

static void testPursuitCommandsLeadAMovingTarget()
{
    PursuitWorldState stationary = makeControlWorld();
    PursuitWorldState moving = stationary;
    moving.target.vx = 0.02;
    moving.target.vy = -0.01;
    PursuitRoleMap roles;
    assignPursuitRoles({2, 4, 8, 17}, roles);
    PursuitSlotAssigner stationarySlots;
    PursuitSlotAssigner movingSlots;
    WheelCommandSmoother stationarySmoother(1000);
    WheelCommandSmoother movingSmoother(1000);
    const auto first = computePursuitCommands(
        stationary, roles, PursuitPhase::Pursuit, stationarySlots, stationarySmoother);
    const auto second = computePursuitCommands(
        moving, roles, PursuitPhase::Pursuit, movingSlots, movingSmoother);
    bool differs = false;
    for (unsigned int id : roles.pursuerIds)
        differs = differs || first.commands.at(id).left != second.commands.at(id).left ||
                  first.commands.at(id).right != second.commands.at(id).right;
    if (!differs) {
        std::cerr << "moving-target-not-predicted failed\n";
        ++failures;
    }
}

static PursuitRobotState testRobot(unsigned int id, double x, double y, uint64_t feedbackMs)
{
    return {id, {x, y, 0.0}, feedbackMs, true, true, 0.0, 0.0};
}

static std::vector<PursuitRobotState> testFleet(uint64_t feedbackMs)
{
    return {
        testRobot(17, 0.48, 0.50, feedbackMs),
        testRobot(2, 0.80, 0.50, feedbackMs),
        testRobot(29, 1.50, 0.80, feedbackMs),
        testRobot(8, 0.64, 0.78, feedbackMs),
        testRobot(4, 0.64, 0.22, feedbackMs)
    };
}

struct ClosedLoopResult
{
    PursuitPhase phase = PursuitPhase::Idle;
    PursuitFault fault = PursuitFault::None;
    bool sawSurround = false;
    bool sawCapture = false;
    double elapsedSeconds = 0.0;
    double minimumTargetPursuerDistance = std::numeric_limits<double>::infinity();
    double minimumPursuerDistance = std::numeric_limits<double>::infinity();
    std::array<double, 3> finalTargetDistances{{0.0, 0.0, 0.0}};
};

static ClosedLoopResult simulateClosedLoop(
    std::vector<PursuitRobotState> fleet,
    double fieldWidth,
    double fieldHeight,
    int maximumTicks = 1500)
{
    ClosedLoopResult result;
    ZooidTestMode mode;
    uint64_t nowMs = 1000;
    for (auto& robot : fleet) robot.feedbackMs = nowMs;
    if (!mode.start(fleet, nowMs)) {
        result.fault = mode.statusSnapshot().fault;
        return result;
    }
    const PursuitRoleMap roles = mode.roleMap();

    const auto updateClearances = [&]() {
        const auto findPose = [&](unsigned int id) -> const PursuitPose* {
            for (const auto& robot : fleet)
                if (robot.id == id) return &robot.pose;
            return nullptr;
        };
        const PursuitPose* targetPose = findPose(roles.targetId);
        if (targetPose == nullptr) return;
        for (std::size_t i = 0; i < roles.pursuerIds.size(); ++i)
        {
            const PursuitPose* first = findPose(roles.pursuerIds[i]);
            if (first == nullptr) continue;
            result.finalTargetDistances[i] = distanceBetween(*targetPose, *first);
            result.minimumTargetPursuerDistance = std::min(
                result.minimumTargetPursuerDistance,
                result.finalTargetDistances[i]);
            for (std::size_t j = i + 1; j < roles.pursuerIds.size(); ++j)
            {
                const PursuitPose* second = findPose(roles.pursuerIds[j]);
                if (second != nullptr)
                    result.minimumPursuerDistance = std::min(
                        result.minimumPursuerDistance,
                        distanceBetween(*first, *second));
            }
        }
    };
    updateClearances();

    constexpr double timeStep = 0.020;
    for (int tick = 1; tick <= maximumTicks; ++tick)
    {
        nowMs += 20;
        for (auto& robot : fleet) robot.feedbackMs = nowMs;
        const PursuitControlOutput output = mode.update(
            fleet, nowMs, fieldWidth, fieldHeight, static_cast<uint64_t>(tick));
        result.phase = output.phase;
        result.fault = output.fault;
        result.sawSurround = result.sawSurround ||
            output.phase == PursuitPhase::Surround;
        result.sawCapture = result.sawCapture ||
            output.phase == PursuitPhase::Capture;
        result.elapsedSeconds = static_cast<double>(tick) * timeStep;
        if (output.fault != PursuitFault::None ||
            output.phase == PursuitPhase::Captured)
            return result;

        for (auto& robot : fleet)
        {
            const auto found = output.commands.find(robot.id);
            if (found == output.commands.end()) continue;
            const double left = found->second.left /
                PursuitDriveCalibration::WheelUnitsPerMetre;
            const double right = found->second.right /
                PursuitDriveCalibration::WheelUnitsPerMetre;
            const double linear = 0.5 * (left + right);
            const double angular = (right - left) /
                PursuitDriveCalibration::WheelBaseMetres;
            robot.pose.x += linear * std::cos(robot.pose.yaw) * timeStep;
            robot.pose.y += linear * std::sin(robot.pose.yaw) * timeStep;
            robot.pose.yaw = normalizeAngle(robot.pose.yaw + angular * timeStep);
            robot.pose.x = std::max(PursuitProfile::BoundaryMargin,
                std::min(fieldWidth - PursuitProfile::BoundaryMargin, robot.pose.x));
            robot.pose.y = std::max(PursuitProfile::BoundaryMargin,
                std::min(fieldHeight - PursuitProfile::BoundaryMargin, robot.pose.y));
        }
        updateClearances();
    }
    return result;
}

static void testDuplicateFleetFeedbackDoesNotAdvanceMission()
{
    const PursuitPose target{0.73, 0.457, 0.0};
    const auto ring = makeTriangularRing(target, PursuitProfile::PursuitHoldRadius, 0.0);
    std::vector<PursuitRobotState> fleet{
        testRobot(1, target.x, target.y, 1000),
        testRobot(2, ring[0].x, ring[0].y, 1000),
        testRobot(3, ring[1].x, ring[1].y, 1000),
        testRobot(4, ring[2].x, ring[2].y, 1000)
    };
    ZooidTestMode mode;
    mode.start(fleet, 1000);
    for (uint64_t sequence = 1; sequence <= 100; ++sequence)
        mode.update(fleet, 1100, 1.46, 0.914, sequence);
    if (mode.statusSnapshot().phase != PursuitPhase::Pursuit) {
        std::cerr << "duplicate-fleet-feedback-advanced-mission failed\n";
        ++failures;
        return;
    }

    PursuitControlOutput output;
    uint64_t nowMs = 1100;
    for (uint64_t sequence = 101; sequence <= 140; ++sequence)
    {
        nowMs += 20;
        for (auto& robot : fleet) robot.feedbackMs = nowMs;
        output = mode.update(fleet, nowMs, 1.46, 0.914, sequence);
    }
    if (output.phase != PursuitPhase::Surround) {
        std::cerr << "fresh-fleet-feedback-did-not-advance failed\n";
        ++failures;
    }
}

static void testPartialFleetFeedbackCountsOnlyCompleteSets()
{
    const PursuitPose target{0.73, 0.457, 0.0};
    const auto ring = makeTriangularRing(target, PursuitProfile::PursuitHoldRadius, 0.0);
    std::vector<PursuitRobotState> fleet{
        testRobot(1, target.x, target.y, 1000),
        testRobot(2, ring[0].x, ring[0].y, 1001),
        testRobot(3, ring[1].x, ring[1].y, 1002),
        testRobot(4, ring[2].x, ring[2].y, 1003)
    };
    ZooidTestMode mode;
    mode.start(fleet, 1003);
    uint64_t sequence = 1;
    uint64_t stamp = 1003;
    for (int cycle = 0; cycle < 10; ++cycle)
    {
        for (std::size_t index = 0; index < fleet.size(); ++index)
        {
            fleet[index].feedbackMs = ++stamp;
            mode.update(fleet, stamp, 1.46, 0.914, sequence++);
        }
    }
    if (mode.statusSnapshot().phase != PursuitPhase::Pursuit) {
        std::cerr << "partial-fleet-feedback-overcounted failed\n";
        ++failures;
    }
}

static void testCommandHoldStopsBeforeFeedbackFault()
{
    auto fleet = testFleet(1000);
    ZooidTestMode mode;
    mode.start(fleet, 1000);
    for (auto& robot : fleet) robot.feedbackMs = 1020;
    const PursuitControlOutput moving = mode.update(fleet, 1020, 1.90, 1.00, 1);
    const PursuitControlOutput held = mode.update(fleet, 1099, 1.90, 1.00, 2);
    bool changedDuringHold = moving.commands.size() != held.commands.size();
    for (const auto& entry : moving.commands)
    {
        const auto found = held.commands.find(entry.first);
        changedDuringHold = changedDuringHold || found == held.commands.end() ||
            found->second.left != entry.second.left ||
            found->second.right != entry.second.right;
    }
    if (changedDuringHold) {
        std::cerr << "duplicate-feedback-changed-command failed\n";
        ++failures;
    }

    const PursuitControlOutput stopped = mode.update(fleet, 1120, 1.90, 1.00, 3);
    for (const auto& entry : stopped.commands) {
        if (entry.second.left != 0 || entry.second.right != 0) {
            std::cerr << "command-hold-timeout-did-not-stop failed\n";
            ++failures;
            break;
        }
    }
}

static void testLateFleetCatchUpDoesNotRestartStaleCommand()
{
    auto fleet = testFleet(1000);
    ZooidTestMode mode;
    mode.start(fleet, 1000);
    for (auto& robot : fleet) robot.feedbackMs = 1020;
    mode.update(fleet, 1020, 1.90, 1.00, 1);

    // Three logical participants advance while the fourth is delayed. When
    // the last packet finally arrives, the other three are already 100 ms
    // old: this is a complete set, but not a timely set and must stay stopped.
    for (auto& robot : fleet)
        if (robot.id != 17) robot.feedbackMs = 1040;
    mode.update(fleet, 1040, 1.90, 1.00, 2);
    for (auto& robot : fleet)
        if (robot.id == 17) robot.feedbackMs = 1140;
    const PursuitControlOutput stopped = mode.update(
        fleet, 1140, 1.90, 1.00, 3);

    for (const auto& entry : stopped.commands) {
        if (entry.second.left != 0 || entry.second.right != 0) {
            std::cerr << "late-fleet-catch-up-restarted-stale-command failed\n";
            ++failures;
            break;
        }
    }
    if (mode.statusSnapshot().phase != PursuitPhase::Pursuit) {
        std::cerr << "late-fleet-catch-up-advanced-mission failed\n";
        ++failures;
    }
}

static void testClosedLoopCapturesFromWallAndClusterLayouts()
{
    std::vector<PursuitRobotState> wallFleet{
        testRobot(1, 0.08, 0.457, 1000),
        testRobot(2, 0.35, 0.457, 1000),
        testRobot(3, 0.30, 0.70, 1000),
        testRobot(4, 0.30, 0.20, 1000)
    };
    wallFleet[1].pose.yaw = 3.14159265358979323846;
    wallFleet[2].pose.yaw = -2.4;
    wallFleet[3].pose.yaw = 2.4;
    const ClosedLoopResult wall = simulateClosedLoop(wallFleet, 1.46, 0.914);
    if (wall.fault != PursuitFault::None || wall.phase != PursuitPhase::Captured ||
        !wall.sawSurround || !wall.sawCapture || wall.elapsedSeconds > 30.0 ||
        wall.minimumTargetPursuerDistance < 0.09 ||
        wall.minimumPursuerDistance < 0.08) {
        std::cerr << "wall-layout-closed-loop-capture failed\n";
        ++failures;
    }

    std::vector<PursuitRobotState> clusterFleet{
        testRobot(1, 0.73, 0.457, 1000),
        testRobot(2, 0.20, 0.357, 1000),
        testRobot(3, 0.20, 0.457, 1000),
        testRobot(4, 0.20, 0.557, 1000)
    };
    const ClosedLoopResult cluster = simulateClosedLoop(clusterFleet, 1.46, 0.914);
    if (cluster.fault != PursuitFault::None ||
        cluster.phase != PursuitPhase::Captured ||
        !cluster.sawSurround || !cluster.sawCapture ||
        cluster.elapsedSeconds > 30.0 ||
        cluster.minimumTargetPursuerDistance < 0.09 ||
        cluster.minimumPursuerDistance < 0.08) {
        std::cerr << "cluster-layout-closed-loop-capture failed\n";
        ++failures;
    }

    std::vector<PursuitRobotState> blockedWallFleet{
        testRobot(1, 0.08, 0.457, 1000),
        testRobot(2, 0.18, 0.457, 1000),
        testRobot(3, 0.35, 0.70, 1000),
        testRobot(4, 0.35, 0.20, 1000)
    };
    blockedWallFleet[1].pose.yaw = 3.14159265358979323846;
    blockedWallFleet[2].pose.yaw = -2.4;
    blockedWallFleet[3].pose.yaw = 2.4;
    const ClosedLoopResult blockedWall = simulateClosedLoop(
        blockedWallFleet, 1.46, 0.914, 2000);
    if (blockedWall.fault != PursuitFault::None ||
        blockedWall.phase != PursuitPhase::Captured ||
        !blockedWall.sawSurround || !blockedWall.sawCapture ||
        blockedWall.elapsedSeconds > 40.0 ||
        blockedWall.minimumTargetPursuerDistance < 0.08 ||
        blockedWall.minimumPursuerDistance < 0.08) {
        std::cerr << "blocked-wall-closed-loop-capture failed phase="
                  << static_cast<int>(blockedWall.phase)
                  << " fault=" << static_cast<int>(blockedWall.fault)
                  << " elapsed=" << blockedWall.elapsedSeconds
                  << " target-clearance=" << blockedWall.minimumTargetPursuerDistance
                  << " pursuer-clearance=" << blockedWall.minimumPursuerDistance
                  << " final-radii=" << blockedWall.finalTargetDistances[0]
                  << ',' << blockedWall.finalTargetDistances[1]
                  << ',' << blockedWall.finalTargetDistances[2]
                  << "\n";
        ++failures;
    }
}

static void testMissionMapsArbitraryIdsAndZerosExtraRobots()
{
    ZooidTestMode mode;
    auto fleet = testFleet(1000);
    if (!mode.start(fleet, 1000)) {
        std::cerr << "mission-start-with-four failed\n";
        ++failures;
        return;
    }
    const PursuitRoleMap roles = mode.roleMap();
    if (roles.targetId != 2 || roles.pursuerIds != std::array<unsigned int, 3>{{4, 8, 17}}) {
        std::cerr << "mission-role-map failed\n";
        ++failures;
    }
    for (auto& robot : fleet) robot.feedbackMs = 1020;
    const PursuitControlOutput output = mode.update(fleet, 1020, 1.90, 1.00, 1);
    if (output.commands.size() != 5 || output.commands.at(29).left != 0 ||
        output.commands.at(29).right != 0) {
        std::cerr << "extra-robot-not-zero failed\n";
        ++failures;
    }
}

static void testMissionRejectsTooFewAndFaultsAtStaleBoundary()
{
    ZooidTestMode tooFew;
    auto three = testFleet(1000);
    three.resize(3);
    if (tooFew.start(three, 1000) ||
        tooFew.statusSnapshot().fault != PursuitFault::NotEnoughRobots) {
        std::cerr << "mission-too-few failed\n";
        ++failures;
    }

    ZooidTestMode mode;
    const auto fleet = testFleet(1000);
    mode.start(fleet, 1000);
    if (mode.update(fleet, 1249, 1.90, 1.00, 1).fault != PursuitFault::None) {
        std::cerr << "feedback-249ms-considered-stale failed\n";
        ++failures;
    }
    if (mode.update(fleet, 1250, 1.90, 1.00, 2).fault != PursuitFault::FeedbackStale) {
        std::cerr << "feedback-250ms-not-stale failed\n";
        ++failures;
    }
}

static void testMissionFreezesRolesAndDoesNotReplaceMissingParticipant()
{
    ZooidTestMode mode;
    auto fleet = testFleet(1000);
    mode.start(fleet, 1000);
    fleet.push_back(testRobot(1, 1.20, 0.50, 1100));
    for (auto& robot : fleet) robot.feedbackMs = 1100;
    mode.update(fleet, 1100, 1.90, 1.00, 1);
    if (mode.roleMap().targetId != 2) {
        std::cerr << "late-lower-id-reassigned-target failed\n";
        ++failures;
    }
    fleet.erase(std::remove_if(fleet.begin(), fleet.end(), [](const PursuitRobotState& robot) {
        return robot.id == 4;
    }), fleet.end());
    if (mode.update(fleet, 1200, 1.90, 1.00, 2).fault !=
        PursuitFault::ParticipantMissing) {
        std::cerr << "newcomer-replaced-missing-participant failed\n";
        ++failures;
    }
}

static void testMissionStopIsIdempotentAndRestartRebuildsRoles()
{
    ZooidTestMode mode;
    auto fleet = testFleet(1000);
    mode.start(fleet, 1000);
    mode.stop();
    mode.stop();
    const auto stopped = mode.update(fleet, 1100, 1.90, 1.00, 1);
    for (const auto& entry : stopped.commands) {
        if (entry.second.left != 0 || entry.second.right != 0) {
            std::cerr << "repeated-stop-not-zero failed\n";
            ++failures;
        }
    }
    fleet.erase(std::remove_if(fleet.begin(), fleet.end(), [](const PursuitRobotState& robot) {
        return robot.id == 2;
    }), fleet.end());
    fleet.push_back(testRobot(1, 0.80, 0.50, 1200));
    for (auto& robot : fleet) robot.feedbackMs = 1200;
    if (!mode.start(fleet, 1200) || mode.roleMap().targetId != 1) {
        std::cerr << "restart-did-not-rebuild-role-map failed\n";
        ++failures;
    }
}

int main()
{
    testSpeedCodec();
    testTargetSnapshot();
    testRandomHardwareIdsAssignFourLogicalRoles();
    testRoleAssignmentRejectsTooFewWithoutMutation();
    testTriangularRingUsesHandCheckedGeometry();
    testCaptureGeometryChecksRadiusAndAngularContainment();
    testSlotAssignmentsPersistAndNeverExpand();
    testSurroundSlotsRestoreNominalRadius();
    testStateMachineCompletesThreeFreshObservationStages();
    testDuplicateObservationDoesNotAdvanceDwellEvidence();
    testBrokenSurroundFallsBackToPursuit();
    testBrokenAcquiredCaptureFallsBackByDistance();
    testCaptureAcquisitionEscapesBackToPursuit();
    testReceiverHeadingAndPointController();
    testDifferentialDriveSignsLimitsAndSlew();
    testFieldBoundaryBrakePreservesTurning();
    testTargetEscapeRespectsPhaseSpeedAndTerminalStop();
    testCapturePressureSlowsTargetAndEdgePursuitRemainsFeasible();
    testCoordinatedControllerUsesRealIdsAndStopsWhenCaptured();
    testPursuitCommandsLeadAMovingTarget();
    testDuplicateFleetFeedbackDoesNotAdvanceMission();
    testPartialFleetFeedbackCountsOnlyCompleteSets();
    testCommandHoldStopsBeforeFeedbackFault();
    testLateFleetCatchUpDoesNotRestartStaleCommand();
    testClosedLoopCapturesFromWallAndClusterLayouts();
    testMissionMapsArbitraryIdsAndZerosExtraRobots();
    testMissionRejectsTooFewAndFaultsAtStaleBoundary();
    testMissionFreezesRolesAndDoesNotReplaceMissingParticipant();
    testMissionStopIsIdempotentAndRestartRebuildsRoles();
    if (failures != 0) return EXIT_FAILURE;
    std::cout << "zooid core tests passed\n";
    return EXIT_SUCCESS;
}
