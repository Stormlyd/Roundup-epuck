#include "../manager/ZooidTestMode.h"
#include "../manager/ZooidCoordinates.h"
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

static void testConfirmedFieldAndCoordinateMapping()
{
    if (!near(ZooidFieldWidth, 1.460) || !near(ZooidFieldHeight, 0.914)) {
        std::cerr << "confirmed-field-constants failed\n";
        ++failures;
    }
    if (ZooidRawMinX != 63 || ZooidRawMaxX != 960 ||
        ZooidRawMinY != 229 || ZooidRawMaxY != 795) {
        std::cerr << "confirmed-raw-endpoints failed\n";
        ++failures;
    }

    struct CoordinateCase
    {
        uint16_t rawX;
        uint16_t rawY;
        double worldX;
        double worldY;
    };
    const CoordinateCase corners[] = {
        {63, 229, 1.460, 0.0},
        {63, 795, 1.460, 0.914},
        {960, 229, 0.0, 0.0},
        {960, 795, 0.0, 0.914}
    };
    for (const CoordinateCase& corner : corners) {
        ZooidWorldPoint world{0.0, 0.0};
        if (!zooidRawToWorld(corner.rawX, corner.rawY, world) ||
            !near(world.x, corner.worldX) || !near(world.y, corner.worldY)) {
            std::cerr << "raw-endpoint-mapping failed\n";
            ++failures;
        }
    }

    const ZooidWorldPoint midpoint{ZooidFieldWidth / 2.0, ZooidFieldHeight / 2.0};
    uint16_t midpointRawX = 0;
    uint16_t midpointRawY = 0;
    if (!zooidWorldToRaw(midpoint, midpointRawX, midpointRawY) ||
        midpointRawX != 512 || midpointRawY != 512) {
        std::cerr << "world-midpoint-mapping failed\n";
        ++failures;
    }

    ZooidWorldPoint rejectedWorld{0.0, 0.0};
    if (zooidRawToWorld(62, 229, rejectedWorld)) {
        std::cerr << "out-of-range-raw-coordinate accepted\n";
        ++failures;
    }
    uint16_t rejectedRawX = 0;
    uint16_t rejectedRawY = 0;
    if (zooidWorldToRaw({-0.001, 0.0}, rejectedRawX, rejectedRawY)) {
        std::cerr << "out-of-range-world-coordinate accepted\n";
        ++failures;
    }
    if (zooidWorldToRaw({std::numeric_limits<double>::quiet_NaN(), 0.0},
                        rejectedRawX, rejectedRawY) ||
        zooidWorldToRaw({0.0, std::numeric_limits<double>::infinity()},
                        rejectedRawX, rejectedRawY)) {
        std::cerr << "non-finite-world-coordinate accepted\n";
        ++failures;
    }

    const CoordinateCase roundTrips[] = {
        {63, 229, 0.0, 0.0},
        {512, 512, 0.0, 0.0},
        {960, 795, 0.0, 0.0}
    };
    for (const CoordinateCase& source : roundTrips) {
        ZooidWorldPoint world{0.0, 0.0};
        uint16_t rawX = 0;
        uint16_t rawY = 0;
        if (!zooidRawToWorld(source.rawX, source.rawY, world) ||
            !zooidWorldToRaw(world, rawX, rawY) ||
            std::abs(static_cast<int>(rawX) - static_cast<int>(source.rawX)) > 1 ||
            std::abs(static_cast<int>(rawY) - static_cast<int>(source.rawY)) > 1) {
            std::cerr << "coordinate-round-trip failed\n";
            ++failures;
        }
    }

    const PursuitWorldState defaultWorld;
    if (!near(defaultWorld.fieldWidth, 1.460) ||
        !near(defaultWorld.fieldHeight, 0.914)) {
        std::cerr << "pursuit-world-field-defaults failed\n";
        ++failures;
    }
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
    if (straight.left != 20 || straight.right != 20) {
        std::cerr << "differential-straight failed\n";
        ++failures;
    }
    const WheelCommand leftTurn = differentialDrive({0.020, 0.4});
    if (leftTurn.left != 10 || leftTurn.right != 30) {
        std::cerr << "differential-positive-turn failed\n";
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
    bool hasAcceleratedPursuer = false;
    bool pursuerExceededFivefoldCeiling = false;
    for (unsigned int id : roles.pursuerIds) {
        const WheelCommand command = running.commands.at(id);
        const int forwardAverage =
            (static_cast<int>(command.left) + static_cast<int>(command.right)) / 2;
        hasAcceleratedPursuer = hasAcceleratedPursuer || forwardAverage > 30;
        pursuerExceededFivefoldCeiling = pursuerExceededFivefoldCeiling ||
            forwardAverage > 50;
    }
    if (!hasAcceleratedPursuer) {
        std::cerr << "pursuer-fivefold-speed-not-applied failed\n";
        ++failures;
    }
    if (pursuerExceededFivefoldCeiling) {
        std::cerr << "pursuer-fivefold-speed-ceiling failed\n";
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

static void testMissionMapsArbitraryIdsAndZerosExtraRobots()
{
    ZooidTestMode mode;
    const auto fleet = testFleet(1000);
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
    const PursuitControlOutput output = mode.update(fleet, 1100, 1.90, 1.00, 1);
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
    if (mode.update(fleet, 1499, 1.90, 1.00, 1).fault != PursuitFault::None) {
        std::cerr << "feedback-499ms-considered-stale failed\n";
        ++failures;
    }
    if (mode.update(fleet, 1500, 1.90, 1.00, 2).fault != PursuitFault::FeedbackStale) {
        std::cerr << "feedback-500ms-not-stale failed\n";
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
    testConfirmedFieldAndCoordinateMapping();
    testTriangularRingUsesHandCheckedGeometry();
    testCaptureGeometryChecksRadiusAndAngularContainment();
    testSlotAssignmentsPersistAndNeverExpand();
    testStateMachineCompletesThreeFreshObservationStages();
    testDuplicateObservationDoesNotAdvanceDwellEvidence();
    testBrokenSurroundFallsBackToPursuit();
    testBrokenAcquiredCaptureFallsBackByDistance();
    testCaptureAcquisitionEscapesBackToPursuit();
    testReceiverHeadingAndPointController();
    testDifferentialDriveSignsLimitsAndSlew();
    testTargetEscapeRespectsPhaseSpeedAndTerminalStop();
    testCapturePressureSlowsTargetAndEdgePursuitRemainsFeasible();
    testCoordinatedControllerUsesRealIdsAndStopsWhenCaptured();
    testPursuitCommandsLeadAMovingTarget();
    testMissionMapsArbitraryIdsAndZerosExtraRobots();
    testMissionRejectsTooFewAndFaultsAtStaleBoundary();
    testMissionFreezesRolesAndDoesNotReplaceMissingParticipant();
    testMissionStopIsIdempotentAndRestartRebuildsRoles();
    if (failures != 0) return EXIT_FAILURE;
    std::cout << "zooid core tests passed\n";
    return EXIT_SUCCESS;
}
