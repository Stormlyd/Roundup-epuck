#include "ZooidPursuitGeometry.h"
#include "ZooidPursuitSlotPolicy.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double Pi = 3.14159265358979323846;

bool finitePose(const PursuitPose& pose)
{
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

bool finitePolicyInput(const PursuitWorldState& world, double radius)
{
    if (!finitePose(world.target.pose) || !std::isfinite(world.target.vx) ||
        !std::isfinite(world.target.vy) || !std::isfinite(world.fieldWidth) ||
        !std::isfinite(world.fieldHeight) || !std::isfinite(radius))
        return false;
    for (const PursuitRobotState& pursuer : world.pursuers)
        if (!finitePose(pursuer.pose) || !std::isfinite(pursuer.vx) ||
            !std::isfinite(pursuer.vy))
            return false;
    return true;
}

struct SlotPermutation
{
    std::array<int, 3> order{{0, 0, 0}};
    int index = 0;
};

SlotPermutation bestPermutation(const PursuitWorldState& world,
                                const std::array<PursuitPose, 3>& ring)
{
    SlotPermutation best;
    int initialHeading = 0;
    decodePursuitSlotAction(0, initialHeading, best.order);
    double bestCost = std::numeric_limits<double>::infinity();
    for (int index = 0; index < PursuitSlotPermutationCount; ++index)
    {
        int headingIndex = 0;
        std::array<int, 3> order;
        if (!decodePursuitSlotAction(index, headingIndex, order)) continue;
        double cost = 0.0;
        for (std::size_t i = 0; i < 3; ++i)
        {
            const double d = distanceBetween(world.pursuers[i].pose, ring[order[i]]);
            cost += d * d;
        }
        if (cost < bestCost)
        {
            bestCost = cost;
            best.order = order;
            best.index = index;
        }
    }
    return best;
}
}

double normalizeAngle(double angle)
{
    while (angle > Pi) angle -= 2.0 * Pi;
    while (angle <= -Pi) angle += 2.0 * Pi;
    return angle;
}

double distanceBetween(const PursuitPose& first, const PursuitPose& second)
{
    return std::hypot(first.x - second.x, first.y - second.y);
}

std::array<PursuitPose, 3> makeTriangularRing(const PursuitPose& target,
                                              double radius,
                                              double headingOffset)
{
    std::array<PursuitPose, 3> ring;
    for (std::size_t i = 0; i < ring.size(); ++i)
    {
        const double bearing = headingOffset + static_cast<double>(i) * 2.0 * Pi / 3.0;
        ring[i] = {target.x + radius * std::cos(bearing),
                   target.y + radius * std::sin(bearing),
                   normalizeAngle(bearing + Pi)};
    }
    return ring;
}

std::array<double, 3> circularAngularGaps(
    const PursuitPose& target,
    const std::array<PursuitPose, 3>& poses)
{
    std::array<double, 3> angles;
    for (std::size_t i = 0; i < poses.size(); ++i)
        angles[i] = std::atan2(poses[i].y - target.y, poses[i].x - target.x);
    std::sort(angles.begin(), angles.end());
    return {{angles[1] - angles[0],
             angles[2] - angles[1],
             angles[0] + 2.0 * Pi - angles[2]}};
}

bool ringInsideBounds(const std::array<PursuitPose, 3>& ring,
                      double width,
                      double height,
                      double margin)
{
    if (!std::isfinite(width) || !std::isfinite(height) || !std::isfinite(margin) ||
        width <= 2.0 * margin || height <= 2.0 * margin)
        return false;
    for (const PursuitPose& pose : ring)
    {
        if (!finitePose(pose) || pose.x < margin || pose.x > width - margin ||
            pose.y < margin || pose.y > height - margin)
            return false;
    }
    return true;
}

bool captureGeometrySatisfied(const PursuitPose& target,
                              const std::array<PursuitPose, 3>& pursuers,
                              double captureRadius,
                              double minAngleGap,
                              double maxAngleGap)
{
    for (const PursuitPose& pose : pursuers)
        if (distanceBetween(target, pose) > captureRadius)
            return false;
    const auto gaps = circularAngularGaps(target, pursuers);
    return *std::min_element(gaps.begin(), gaps.end()) >= minAngleGap &&
           *std::max_element(gaps.begin(), gaps.end()) < maxAngleGap;
}

bool surroundGeometrySatisfied(const PursuitPose& target,
                               const std::array<PursuitPose, 3>& pursuers,
                               double radius,
                               double tolerance,
                               double minAngleGap,
                               double maxAngleGap)
{
    for (const PursuitPose& pose : pursuers)
        if (std::abs(distanceBetween(target, pose) - radius) > tolerance)
            return false;
    const auto gaps = circularAngularGaps(target, pursuers);
    return *std::min_element(gaps.begin(), gaps.end()) >= minAngleGap &&
           *std::max_element(gaps.begin(), gaps.end()) < maxAngleGap;
}

bool PursuitSlotAssigner::assign(const PursuitWorldState& world,
                                 PursuitPhase phase,
                                 double radius,
                                 bool nonExpanding,
                                 std::array<PursuitPose, 3>& goals)
{
    bool found = false;
    if (remembered_ && phase_ == phase)
    {
        for (std::size_t i = 0; i < 3; ++i)
            goals[i] = makeTriangularRing(world.target.pose, radius, bearings_[i])[0];
        found = ringInsideBounds(
            goals,
            rememberedFromPolicy_ ? ZooidFieldWidth : world.fieldWidth,
            rememberedFromPolicy_ ? ZooidFieldHeight : world.fieldHeight,
            0.057);
    }

    if (!found)
    {
        if (policy_ != nullptr && finitePolicyInput(world, radius))
        {
            PursuitSlotObservation observation;
            observation.world = world;
            observation.phase = phase;
            observation.radius = radius;
            observation.previousAction = previousAction_;
            int action = -1;
            int headingIndex = 0;
            std::array<int, 3> order;
            if (policy_->chooseAction(observation, action) &&
                decodePursuitSlotAction(action, headingIndex, order))
            {
                const auto ring = makeTriangularRing(
                    world.target.pose, radius,
                    static_cast<double>(headingIndex) * 2.0 * Pi /
                    PursuitSlotHeadingCount);
                if (ringInsideBounds(
                        ring, ZooidFieldWidth, ZooidFieldHeight, 0.057))
                {
                    for (std::size_t i = 0; i < 3; ++i)
                    {
                        goals[i] = ring[order[i]];
                        bearings_[i] = std::atan2(
                            goals[i].y - world.target.pose.y,
                            goals[i].x - world.target.pose.x);
                    }
                    phase_ = phase;
                    remembered_ = true;
                    rememberedFromPolicy_ = true;
                    previousAction_ = action;
                    found = true;
                }
            }
        }

        if (!found)
        {
            double bestCost = std::numeric_limits<double>::infinity();
            for (int sample = 0; sample < PursuitSlotHeadingCount; ++sample)
            {
                const double heading = static_cast<double>(sample) * 2.0 * Pi /
                    PursuitSlotHeadingCount;
                const auto ring = makeTriangularRing(world.target.pose, radius, heading);
                if (!ringInsideBounds(ring, world.fieldWidth, world.fieldHeight, 0.057))
                    continue;
                const SlotPermutation permutation = bestPermutation(world, ring);
                double cost = 0.0;
                for (std::size_t i = 0; i < 3; ++i)
                {
                    const double d = distanceBetween(
                        world.pursuers[i].pose, ring[permutation.order[i]]);
                    cost += d * d;
                }
                if (cost < bestCost)
                {
                    bestCost = cost;
                    for (std::size_t i = 0; i < 3; ++i)
                        goals[i] = ring[permutation.order[i]];
                    previousAction_ = sample * PursuitSlotPermutationCount +
                        permutation.index;
                    found = true;
                }
            }
            if (!found)
                return false;
            phase_ = phase;
            remembered_ = true;
            rememberedFromPolicy_ = false;
            for (std::size_t i = 0; i < 3; ++i)
                bearings_[i] = std::atan2(goals[i].y - world.target.pose.y,
                                          goals[i].x - world.target.pose.x);
        }
    }

    if (nonExpanding)
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            const double goalRadius = std::min(
                radius, distanceBetween(world.target.pose, world.pursuers[i].pose));
            goals[i].x = world.target.pose.x + goalRadius * std::cos(bearings_[i]);
            goals[i].y = world.target.pose.y + goalRadius * std::sin(bearings_[i]);
        }
    }
    return true;
}

void PursuitSlotAssigner::setPolicy(PursuitSlotPolicy* policy)
{
    policy_ = policy;
    remembered_ = false;
    rememberedFromPolicy_ = false;
    previousAction_ = -1;
}

void PursuitSlotAssigner::clear()
{
    phase_ = PursuitPhase::Idle;
    remembered_ = false;
    rememberedFromPolicy_ = false;
    previousAction_ = -1;
}
