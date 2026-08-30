#include "ZooidPursuitControl.h"

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
    double maxLinear = 0.040;
    if (phase == PursuitPhase::Surround) maxLinear = 0.035;
    if (phase == PursuitPhase::Capture)
    {
        const std::array<double, 4> scales{{1.0, 0.80, 0.45, 0.25}};
        int closeCount = 0;
        for (const auto& pursuer : world.pursuers)
            if (distanceBetween(world.target.pose, pursuer.pose) <= 0.201875)
                ++closeCount;
        maxLinear = 0.025 * scales[std::min(3, closeCount)];
    }
    if (phase == PursuitPhase::Pursuit)
    {
        double nearest = 1e9;
        for (const auto& pursuer : world.pursuers)
            nearest = std::min(nearest, distanceBetween(world.target.pose, pursuer.pose));
        if (nearest < 0.31) maxLinear = 0.050;
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
        if (goal.x >= 0.057 && goal.x <= world.fieldWidth - 0.057 &&
            goal.y >= 0.057 && goal.y <= world.fieldHeight - 0.057)
            return driveToward(world.target.pose, goal, maxLinear, 1.6);
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

    output.commands[roles.targetId] = smoother.smooth(
        roles.targetId, differentialDrive(targetEscapeCommand(world, phase)));

    const double radius = phase == PursuitPhase::Pursuit ? 0.31
                        : phase == PursuitPhase::Surround ? 0.24 : 0.1224;
    const double maxLinear = phase == PursuitPhase::Pursuit ? 0.100
                           : phase == PursuitPhase::Surround ? 0.085 : 0.070;
    std::array<PursuitPose, 3> goals;
    PursuitWorldState planningWorld = world;
    if (phase == PursuitPhase::Pursuit)
    {
        planningWorld.target.pose.x = clampValue(
            world.target.pose.x + world.target.vx * 0.8,
            0.057, world.fieldWidth - 0.057);
        planningWorld.target.pose.y = clampValue(
            world.target.pose.y + world.target.vy * 0.8,
            0.057, world.fieldHeight - 0.057);
    }
    if (!slots.assign(planningWorld, phase, radius,
                      phase != PursuitPhase::Capture, goals))
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            const double bearing = std::atan2(
                world.pursuers[i].pose.y - planningWorld.target.pose.y,
                world.pursuers[i].pose.x - planningWorld.target.pose.x);
            const double fallbackRadius = std::min(
                0.31, distanceBetween(planningWorld.target.pose, world.pursuers[i].pose));
            goals[i] = {
                clampValue(planningWorld.target.pose.x + fallbackRadius * std::cos(bearing),
                           0.057, world.fieldWidth - 0.057),
                clampValue(planningWorld.target.pose.y + fallbackRadius * std::sin(bearing),
                           0.057, world.fieldHeight - 0.057),
                normalizeAngle(bearing + Pi)
            };
        }
    }

    for (std::size_t i = 0; i < 3; ++i)
    {
        PursuitTwist twist = driveToward(world.pursuers[i].pose, goals[i], maxLinear);
        const double gx = goals[i].x - world.pursuers[i].pose.x;
        const double gy = goals[i].y - world.pursuers[i].pose.y;
        for (std::size_t j = 0; j < 3; ++j)
        {
            if (i == j) continue;
            const double separation = distanceBetween(world.pursuers[i].pose,
                                                      world.pursuers[j].pose);
            const double px = world.pursuers[j].pose.x - world.pursuers[i].pose.x;
            const double py = world.pursuers[j].pose.y - world.pursuers[i].pose.y;
            if (separation < 0.14 && gx * px + gy * py > 0.0)
                twist.linear = 0.0;
        }
        output.commands[roles.pursuerIds[i]] = smoother.smooth(
            roles.pursuerIds[i], differentialDrive(twist));
    }
    return output;
}
