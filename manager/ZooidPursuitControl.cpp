#include "ZooidPursuitControl.h"

#include "ZooidPursuitProfile.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double Pi = 3.14159265358979323846;

double clampValue(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

int clampStep(int desired, int previous, int maximumDelta)
{
    return std::max(previous - maximumDelta,
                    std::min(previous + maximumDelta, desired));
}

PursuitPose collisionAwareGoal(const PursuitWorldState& world,
                               std::size_t pursuerIndex,
                               const PursuitPose& nominalGoal,
                               PursuitPhase phase)
{
    const PursuitPose& current = world.pursuers[pursuerIndex].pose;
    PursuitPose transitGoal = nominalGoal;

    // Direct chords to a slot on the opposite side can pass through the
    // target. Route in polar coordinates until the bearing is close, then
    // converge radially. Capture uses a smaller, still body-safe clearance.
    const double clearance = phase == PursuitPhase::Capture
        ? PursuitProfile::CaptureTargetTransitClearance
        : PursuitProfile::TargetTransitClearance;
    const double targetDx = current.x - world.target.pose.x;
    const double targetDy = current.y - world.target.pose.y;
    const double currentRadius = std::hypot(targetDx, targetDy);
    const double goalDx = nominalGoal.x - world.target.pose.x;
    const double goalDy = nominalGoal.y - world.target.pose.y;
    const double goalRadius = std::hypot(goalDx, goalDy);
    const double goalBearing = std::atan2(goalDy, goalDx);
    const double currentBearing = currentRadius > 1e-8
        ? std::atan2(targetDy, targetDx) : goalBearing;
    if (currentRadius < clearance)
    {
        transitGoal.x = world.target.pose.x + clearance * std::cos(currentBearing);
        transitGoal.y = world.target.pose.y + clearance * std::sin(currentBearing);
    }
    else
    {
        const double bearingError = normalizeAngle(goalBearing - currentBearing);
        if (std::abs(bearingError) > PursuitProfile::TargetTransitBearingStep)
        {
            const double step = clampValue(
                bearingError,
                -PursuitProfile::TargetTransitBearingStep,
                PursuitProfile::TargetTransitBearingStep);
            const double transitRadius = std::max(
                clearance, goalRadius);
            transitGoal.x = world.target.pose.x +
                transitRadius * std::cos(currentBearing + step);
            transitGoal.y = world.target.pose.y +
                transitRadius * std::sin(currentBearing + step);
        }
    }

    double steerX = transitGoal.x - current.x;
    double steerY = transitGoal.y - current.y;
    const double attractionLength = std::hypot(steerX, steerY);
    if (attractionLength > PursuitProfile::CollisionAttractionLength)
    {
        steerX *= PursuitProfile::CollisionAttractionLength / attractionLength;
        steerY *= PursuitProfile::CollisionAttractionLength / attractionLength;
    }

    for (std::size_t neighborIndex = 0; neighborIndex < 3; ++neighborIndex)
    {
        if (neighborIndex == pursuerIndex) continue;
        double awayX = current.x - world.pursuers[neighborIndex].pose.x;
        double awayY = current.y - world.pursuers[neighborIndex].pose.y;
        double separation = std::hypot(awayX, awayY);
        if (separation >= PursuitProfile::CollisionInfluenceRadius)
            continue;
        if (separation < 1e-6)
        {
            const double angle = static_cast<double>(pursuerIndex) * 2.0 * Pi / 3.0;
            awayX = std::cos(angle);
            awayY = std::sin(angle);
            separation = 0.0;
        }
        else
        {
            awayX /= separation;
            awayY /= separation;
        }

        const double penetration =
            PursuitProfile::CollisionInfluenceRadius - separation;
        steerX += PursuitProfile::CollisionRepulsionGain * penetration * awayX;
        steerY += PursuitProfile::CollisionRepulsionGain * penetration * awayY;
        // A shared counter-clockwise circulation term prevents symmetric
        // head-on layouts from settling at a potential-field deadlock.
        steerX += PursuitProfile::CollisionTangentialGain * penetration * (-awayY);
        steerY += PursuitProfile::CollisionTangentialGain * penetration * awayX;
    }

    if (std::hypot(steerX, steerY) < 1e-8)
        return nominalGoal;
    return {
        clampValue(current.x + steerX,
                   PursuitProfile::BoundaryMargin,
                   world.fieldWidth - PursuitProfile::BoundaryMargin),
        clampValue(current.y + steerY,
                   PursuitProfile::BoundaryMargin,
                   world.fieldHeight - PursuitProfile::BoundaryMargin),
        nominalGoal.yaw
    };
}

PursuitTwist guardedTargetDriveToward(const PursuitWorldState& world,
                                      const PursuitPose& goal,
                                      double maxLinear,
                                      double maxAngular)
{
    PursuitTwist twist = driveToward(
        world.target.pose, goal, maxLinear, maxAngular);
    if (twist.linear <= 0.0)
        return twist;

    const double forwardX = std::cos(world.target.pose.yaw);
    const double forwardY = std::sin(world.target.pose.yaw);
    for (const auto& pursuer : world.pursuers)
    {
        const double dx = pursuer.pose.x - world.target.pose.x;
        const double dy = pursuer.pose.y - world.target.pose.y;
        const double separation = std::hypot(dx, dy);
        if (separation < PursuitProfile::TargetMotionGuardRadius &&
            forwardX * dx + forwardY * dy > 0.0)
        {
            // Hold translation while retaining the turn command. The pursuer
            // has its own polar clearance route and will vacate the corridor.
            twist.linear = 0.0;
            break;
        }
    }
    return twist;
}
}

double receiverHeadingToYaw(double receiverDegrees)
{
    return normalizeAngle((90.0 - receiverDegrees) * Pi / 180.0);
}

PursuitTwist driveToward(const PursuitPose& current,
                         const PursuitPose& goal,
                         double maxLinear,
                         double maxAngular)
{
    if (!std::isfinite(current.x) || !std::isfinite(current.y) ||
        !std::isfinite(current.yaw) || !std::isfinite(goal.x) ||
        !std::isfinite(goal.y) || !std::isfinite(maxLinear) || maxLinear <= 0.0)
        return {};
    const double goalDistance = distanceBetween(current, goal);
    if (goalDistance <= 1e-9)
        return {};
    const double desiredHeading = std::atan2(goal.y - current.y, goal.x - current.x);
    const double error = normalizeAngle(desiredHeading - current.yaw);
    const double alignment = std::max(0.0, std::cos(error));
    const double linear = std::abs(error) > 1.2
        ? 0.0
        : std::min(goalDistance, maxLinear) * alignment * alignment;
    return {linear, clampValue(2.0 * error, -maxAngular, maxAngular)};
}

PursuitTwist brakeAtFieldBoundary(const PursuitPose& current,
                                  double fieldWidth,
                                  double fieldHeight,
                                  PursuitTwist twist)
{
    if (twist.linear <= 0.0 || fieldWidth <= 2.0 * PursuitProfile::BoundaryMargin ||
        fieldHeight <= 2.0 * PursuitProfile::BoundaryMargin)
        return twist;

    const double velocityX = twist.linear * std::cos(current.yaw);
    const double velocityY = twist.linear * std::sin(current.yaw);
    double scale = 1.0;
    if (velocityX < 0.0)
        scale = std::min(scale, (current.x - PursuitProfile::BoundaryMargin) /
                                PursuitProfile::BoundaryBrakingBand);
    if (velocityX > 0.0)
        scale = std::min(scale, (fieldWidth - PursuitProfile::BoundaryMargin - current.x) /
                                PursuitProfile::BoundaryBrakingBand);
    if (velocityY < 0.0)
        scale = std::min(scale, (current.y - PursuitProfile::BoundaryMargin) /
                                PursuitProfile::BoundaryBrakingBand);
    if (velocityY > 0.0)
        scale = std::min(scale, (fieldHeight - PursuitProfile::BoundaryMargin - current.y) /
                                PursuitProfile::BoundaryBrakingBand);
    twist.linear *= clampValue(scale, 0.0, 1.0);
    return twist;
}

WheelCommand differentialDrive(const PursuitTwist& twist,
                               double wheelBase,
                               double unitsPerMetrePerSecond)
{
    if (!std::isfinite(twist.linear) || !std::isfinite(twist.angular) ||
        !std::isfinite(wheelBase) || wheelBase <= 0.0 ||
        !std::isfinite(unitsPerMetrePerSecond) || unitsPerMetrePerSecond <= 0.0)
        return {};
    double left = (twist.linear - twist.angular * wheelBase / 2.0) * unitsPerMetrePerSecond;
    double right = (twist.linear + twist.angular * wheelBase / 2.0) * unitsPerMetrePerSecond;
    const double largest = std::max(std::abs(left), std::abs(right));
    if (largest > 1000.0)
    {
        left *= 1000.0 / largest;
        right *= 1000.0 / largest;
    }
    return {static_cast<int16_t>(std::lround(left)),
            static_cast<int16_t>(std::lround(right))};
}

WheelCommandSmoother::WheelCommandSmoother(int maxDelta)
    : maxDelta_(std::max(1, maxDelta))
{
}

WheelCommand WheelCommandSmoother::smooth(unsigned int id, WheelCommand desired)
{
    const WheelCommand previous = previous_.count(id) ? previous_[id] : WheelCommand{};
    WheelCommand result{
        static_cast<int16_t>(clampStep(desired.left, previous.left, maxDelta_)),
        static_cast<int16_t>(clampStep(desired.right, previous.right, maxDelta_))
    };
    previous_[id] = result;
    return result;
}

void WheelCommandSmoother::clear()
{
    previous_.clear();
}

PursuitTwist targetEscapeCommand(const PursuitWorldState& world,
                                 PursuitPhase phase)
{
    if (phase == PursuitPhase::Idle || phase == PursuitPhase::Captured)
        return {};
    double maxLinear = PursuitProfile::TargetPursuitSpeed;
    if (phase == PursuitPhase::Surround)
        maxLinear = PursuitProfile::TargetSurroundSpeed;
    if (phase == PursuitPhase::Capture)
    {
        const std::array<double, 4> scales{{1.0, 0.80, 0.45, 0.25}};
        int closeCount = 0;
        for (const auto& pursuer : world.pursuers)
            if (distanceBetween(world.target.pose, pursuer.pose) <=
                PursuitProfile::CapturePressureRadius)
                ++closeCount;
        maxLinear = PursuitProfile::TargetCaptureSpeed *
                    scales[std::min(3, closeCount)];
    }
    if (phase == PursuitPhase::Pursuit)
    {
        double nearest = 1e9;
        for (const auto& pursuer : world.pursuers)
            nearest = std::min(nearest, distanceBetween(world.target.pose, pursuer.pose));
        if (nearest < PursuitProfile::PursuitHoldRadius)
            maxLinear = PursuitProfile::TargetPressureSpeed;
    }

    // A full surround ring cannot exist next to a wall or in a corner. Bring
    // the target back into the feasible operating region before it resumes
    // evasive motion; otherwise PURSUIT can remain active forever.
    if (!triangularRingFeasible(world.target.pose,
                                PursuitProfile::SurroundRadius,
                                world.fieldWidth,
                                world.fieldHeight,
                                PursuitProfile::BoundaryMargin))
    {
        const PursuitPose centre{world.fieldWidth * 0.5,
                                 world.fieldHeight * 0.5, 0.0};
        return guardedTargetDriveToward(world, centre, maxLinear, 1.6);
    }

    double awayX = 0.0;
    double awayY = 0.0;
    for (const auto& pursuer : world.pursuers)
    {
        const double dx = world.target.pose.x - pursuer.pose.x;
        const double dy = world.target.pose.y - pursuer.pose.y;
        const double length = std::max(1e-6, std::hypot(dx, dy));
        awayX += dx / length;
        awayY += dy / length;
    }
    if (std::hypot(awayX, awayY) < 1e-6)
    {
        awayX = std::cos(world.target.pose.yaw);
        awayY = std::sin(world.target.pose.yaw);
    }
    const double base = std::atan2(awayY, awayX);
    const std::array<double, 9> turns{{0.0, Pi / 4.0, -Pi / 4.0, Pi / 2.0,
                                      -Pi / 2.0, 3.0 * Pi / 4.0,
                                      -3.0 * Pi / 4.0, Pi, -Pi}};
    for (double turn : turns)
    {
        const double angle = base + turn;
        const PursuitPose goal{world.target.pose.x + 0.18 * std::cos(angle),
                               world.target.pose.y + 0.18 * std::sin(angle), angle};
        if (triangularRingFeasible(goal,
                                   PursuitProfile::SurroundRadius,
                                   world.fieldWidth,
                                   world.fieldHeight,
                                   PursuitProfile::BoundaryMargin))
            return guardedTargetDriveToward(world, goal, maxLinear, 1.6);
    }
    return {};
}

PursuitControlOutput computePursuitCommands(
    const PursuitWorldState& world,
    const PursuitRoleMap& roles,
    PursuitPhase phase,
    PursuitSlotAssigner& slots,
    WheelCommandSmoother& smoother)
{
    PursuitControlOutput output;
    output.phase = phase;
    if (!roles.valid || world.target.id != roles.targetId)
    {
        output.fault = PursuitFault::ParticipantMissing;
        return output;
    }
    for (std::size_t i = 0; i < 3; ++i)
        if (world.pursuers[i].id != roles.pursuerIds[i])
        {
            output.fault = PursuitFault::ParticipantMissing;
            return output;
        }

    const auto finite = [](const PursuitPose& pose) {
        return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
    };
    if (!finite(world.target.pose))
    {
        output.fault = PursuitFault::InvalidFeedback;
        return output;
    }
    for (const auto& pursuer : world.pursuers)
        if (!finite(pursuer.pose))
        {
            output.fault = PursuitFault::InvalidFeedback;
            return output;
        }

    if (phase == PursuitPhase::Idle || phase == PursuitPhase::Captured)
    {
        for (unsigned int id : roles.participantIds()) output.commands[id] = {0, 0};
        return output;
    }

    const PursuitTwist targetTwist = brakeAtFieldBoundary(
        world.target.pose, world.fieldWidth, world.fieldHeight,
        targetEscapeCommand(world, phase));
    output.commands[roles.targetId] = smoother.smooth(
        roles.targetId, differentialDrive(targetTwist));

    const double radius = phase == PursuitPhase::Pursuit
        ? PursuitProfile::PursuitHoldRadius
        : phase == PursuitPhase::Surround
            ? PursuitProfile::SurroundRadius
            : PursuitProfile::CaptureGoalRadius;
    const double maxLinear = phase == PursuitPhase::Pursuit
        ? PursuitProfile::PursuerPursuitSpeed
        : phase == PursuitPhase::Surround
            ? PursuitProfile::PursuerSurroundSpeed
            : PursuitProfile::PursuerCaptureSpeed;
    std::array<PursuitPose, 3> goals;
    PursuitWorldState planningWorld = world;
    if (phase == PursuitPhase::Pursuit)
    {
        planningWorld.target.pose.x = clampValue(
            world.target.pose.x + world.target.vx * 0.8,
            PursuitProfile::BoundaryMargin,
            world.fieldWidth - PursuitProfile::BoundaryMargin);
        planningWorld.target.pose.y = clampValue(
            world.target.pose.y + world.target.vy * 0.8,
            PursuitProfile::BoundaryMargin,
            world.fieldHeight - PursuitProfile::BoundaryMargin);
    }
    if (!slots.assign(planningWorld, phase, radius,
                      phase == PursuitPhase::Pursuit, goals))
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            const double bearing = std::atan2(
                world.pursuers[i].pose.y - planningWorld.target.pose.y,
                world.pursuers[i].pose.x - planningWorld.target.pose.x);
            const double fallbackRadius = std::min(
                PursuitProfile::PursuitHoldRadius,
                distanceBetween(planningWorld.target.pose, world.pursuers[i].pose));
            goals[i] = {
                clampValue(planningWorld.target.pose.x + fallbackRadius * std::cos(bearing),
                           PursuitProfile::BoundaryMargin,
                           world.fieldWidth - PursuitProfile::BoundaryMargin),
                clampValue(planningWorld.target.pose.y + fallbackRadius * std::sin(bearing),
                           PursuitProfile::BoundaryMargin,
                           world.fieldHeight - PursuitProfile::BoundaryMargin),
                normalizeAngle(bearing + Pi)
            };
        }
    }

    for (std::size_t i = 0; i < 3; ++i)
    {
        const PursuitPose steeringGoal = collisionAwareGoal(world, i, goals[i], phase);
        const PursuitTwist twist = brakeAtFieldBoundary(
            world.pursuers[i].pose, world.fieldWidth, world.fieldHeight,
            driveToward(world.pursuers[i].pose, steeringGoal, maxLinear));
        output.commands[roles.pursuerIds[i]] = smoother.smooth(
            roles.pursuerIds[i], differentialDrive(twist));
    }
    return output;
}
