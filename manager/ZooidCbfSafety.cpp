#include "ZooidCbfSafety.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace
{

enum class InputValidity
{
    Valid,
    Invalid,
    Unsafe
};

struct ConstraintTerm
{
    std::size_t index;
    double coefficient;
};

struct Halfspace
{
    std::vector<ConstraintTerm> terms;
    double lowerBound = 0.0;
    double normSquared = 0.0;
};

struct ExtendedValue
{
    double high;
    double low;
};

struct StrictConstraintTerm
{
    std::size_t index;
    ExtendedValue coefficient;
};

struct StrictHalfspace
{
    std::vector<StrictConstraintTerm> terms;
    ExtendedValue lowerBound;
};

ExtendedValue normalizedExtended(double high, double low)
{
    const double sum = high + low;
    return {sum, low - (sum - high)};
}

ExtendedValue addExtended(const ExtendedValue& first,
                          const ExtendedValue& second)
{
    const double high = first.high + second.high;
    const double secondVirtual = high - first.high;
    const double error =
        (first.high - (high - secondVirtual)) +
        (second.high - secondVirtual);
    return normalizedExtended(high, error + first.low + second.low);
}

ExtendedValue exactDifference(double first, double second)
{
    const double high = first - second;
    const double secondVirtual = first - high;
    const double firstVirtual = high + secondVirtual;
    return {high,
            (first - firstVirtual) + (secondVirtual - second)};
}

ExtendedValue exactProduct(double first, double second)
{
    const double high = first * second;
    return {high, std::fma(first, second, -high)};
}

ExtendedValue negateExtended(const ExtendedValue& value)
{
    return {-value.high, -value.low};
}

ExtendedValue subtractExtended(const ExtendedValue& first,
                               const ExtendedValue& second)
{
    return addExtended(first, negateExtended(second));
}

ExtendedValue multiplyExtended(const ExtendedValue& first,
                               const ExtendedValue& second)
{
    ExtendedValue result = exactProduct(first.high, second.high);
    result = addExtended(
        result, exactProduct(first.high, second.low));
    result = addExtended(
        result, exactProduct(first.low, second.high));
    return addExtended(
        result, exactProduct(first.low, second.low));
}

ExtendedValue squareExtended(const ExtendedValue& value)
{
    ExtendedValue result = exactProduct(value.high, value.high);
    result = addExtended(
        result, exactProduct(2.0 * value.high, value.low));
    return addExtended(result, exactProduct(value.low, value.low));
}

bool extendedLess(const ExtendedValue& first,
                  const ExtendedValue& second)
{
    return first.high < second.high ||
        (first.high == second.high && first.low < second.low);
}

bool finiteExtended(const ExtendedValue& value)
{
    return std::isfinite(value.high) && std::isfinite(value.low);
}

double effectiveTolerance(const ZooidCbfConfig& config)
{
    return std::min(config.tolerance, 1e-9);
}

ZooidCbfResult zeroResult(const std::vector<CbfRobotState>& robots,
                         ZooidCbfStatus status)
{
    ZooidCbfResult result;
    result.status = status;
    for (const CbfRobotState& robot : robots) result.commands[robot.id] = {};
    return result;
}

bool validConfig(const ZooidCbfConfig& config)
{
    const bool finite =
        std::isfinite(config.fieldWidth) &&
        std::isfinite(config.fieldHeight) &&
        std::isfinite(config.safeRadius) &&
        std::isfinite(config.minimumDistance) &&
        std::isfinite(config.gamma) &&
        std::isfinite(config.commandUnitsPerMetrePerSecond) &&
        std::isfinite(config.tolerance);
    return finite &&
        config.fieldWidth > 0.0 && config.fieldHeight > 0.0 &&
        config.safeRadius >= 0.0 &&
        config.safeRadius <= config.fieldWidth / 2.0 &&
        config.safeRadius <= config.fieldHeight / 2.0 &&
        config.minimumDistance >= 0.0 && config.gamma >= 0.0 &&
        config.commandUnitsPerMetrePerSecond > 0.0 &&
        config.freshnessLimitMs > 0 &&
        config.maximumWheelCommand >= 0 &&
        config.maximumWheelCommand <= std::numeric_limits<int16_t>::max() &&
        config.maximumIterations > 0 &&
        config.maximumDiscreteSearchNodes > 0 && config.tolerance >= 0.0;
}

InputValidity validateStructure(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& commands,
    const ZooidCbfConfig& config,
    std::vector<CbfRobotState>& sorted)
{
    if (!validConfig(config) || commands.size() != robots.size())
        return InputValidity::Invalid;

    sorted = robots;
    std::sort(sorted.begin(), sorted.end(),
              [](const CbfRobotState& first, const CbfRobotState& second) {
                  return first.id < second.id;
              });
    for (std::size_t i = 0; i < sorted.size(); ++i) {
        const CbfRobotState& robot = sorted[i];
        if ((i > 0 && sorted[i - 1].id == robot.id) ||
            commands.find(robot.id) == commands.end() ||
            !std::isfinite(robot.pose.x) ||
            !std::isfinite(robot.pose.y) ||
            !std::isfinite(robot.pose.yaw)) {
            return InputValidity::Invalid;
        }
    }
    return InputValidity::Valid;
}

bool validTiming(const std::vector<CbfRobotState>& robots,
                 uint64_t nowMs,
                 const ZooidCbfConfig& config)
{
    if (robots.empty()) return true;
    uint64_t earliest = robots.front().feedbackMs;
    uint64_t latest = robots.front().feedbackMs;
    for (const CbfRobotState& robot : robots) {
        if (robot.feedbackMs > nowMs ||
            nowMs - robot.feedbackMs >= config.freshnessLimitMs) {
            return false;
        }
        earliest = std::min(earliest, robot.feedbackMs);
        latest = std::max(latest, robot.feedbackMs);
    }
    return latest - earliest <= config.maximumSnapshotSkewMs;
}

InputValidity validateGeometry(const std::vector<CbfRobotState>& robots,
                               const ZooidCbfConfig& config)
{
    const ExtendedValue safeRadius = {config.safeRadius, 0.0};
    const ExtendedValue maximumX =
        exactDifference(config.fieldWidth, config.safeRadius);
    const ExtendedValue maximumY =
        exactDifference(config.fieldHeight, config.safeRadius);
    for (const CbfRobotState& robot : robots) {
        const ExtendedValue x = {robot.pose.x, 0.0};
        const ExtendedValue y = {robot.pose.y, 0.0};
        if (extendedLess(x, safeRadius) || extendedLess(maximumX, x) ||
            extendedLess(y, safeRadius) || extendedLess(maximumY, y)) {
            return InputValidity::Unsafe;
        }
    }

    const ExtendedValue minimumDistanceSquared = squareExtended(
        {config.minimumDistance, 0.0});
    if (!std::isfinite(minimumDistanceSquared.high) ||
        !std::isfinite(minimumDistanceSquared.low)) {
        return InputValidity::Invalid;
    }
    for (std::size_t i = 0; i < robots.size(); ++i) {
        for (std::size_t j = i + 1; j < robots.size(); ++j) {
            const ExtendedValue dx = exactDifference(
                robots[i].pose.x, robots[j].pose.x);
            const ExtendedValue dy = exactDifference(
                robots[i].pose.y, robots[j].pose.y);
            const ExtendedValue distanceSquared = addExtended(
                squareExtended(dx), squareExtended(dy));
            if (!std::isfinite(distanceSquared.high) ||
                !std::isfinite(distanceSquared.low)) {
                return InputValidity::Invalid;
            }
            if (extendedLess(distanceSquared, minimumDistanceSquared))
                return InputValidity::Unsafe;
        }
    }
    return InputValidity::Valid;
}

bool appendHalfspace(std::vector<Halfspace>& constraints,
                     std::vector<ConstraintTerm> terms,
                     double lowerBound)
{
    Halfspace constraint;
    constraint.terms = std::move(terms);
    constraint.lowerBound = lowerBound;
    if (!std::isfinite(lowerBound)) return false;
    for (const ConstraintTerm& term : constraint.terms) {
        if (!std::isfinite(term.coefficient)) return false;
        constraint.normSquared += term.coefficient * term.coefficient;
    }
    if (!std::isfinite(constraint.normSquared)) {
        return false;
    }
    constraints.push_back(std::move(constraint));
    return true;
}

bool buildConstraints(const std::vector<CbfRobotState>& robots,
                      const std::vector<double>& differentials,
                      const ZooidCbfConfig& config,
                      std::vector<Halfspace>& constraints)
{
    constraints.clear();
    const double wheelLimit = static_cast<double>(config.maximumWheelCommand);
    const double units = config.commandUnitsPerMetrePerSecond;
    for (std::size_t i = 0; i < robots.size(); ++i) {
        const double differential = differentials[i];
        const double cosine = std::cos(robots[i].pose.yaw);
        const double sine = std::sin(robots[i].pose.yaw);
        const double left = robots[i].pose.x - config.safeRadius;
        const double right = config.fieldWidth - config.safeRadius - robots[i].pose.x;
        const double bottom = robots[i].pose.y - config.safeRadius;
        const double top = config.fieldHeight - config.safeRadius - robots[i].pose.y;

        if (!appendHalfspace(constraints, {{i, 1.0}}, 0.0) ||
            !appendHalfspace(constraints, {{i, 1.0}}, differential - wheelLimit) ||
            !appendHalfspace(constraints, {{i, -1.0}}, -wheelLimit - differential) ||
            !appendHalfspace(constraints, {{i, 1.0}}, -wheelLimit - differential) ||
            !appendHalfspace(constraints, {{i, -1.0}}, -wheelLimit + differential) ||
            !appendHalfspace(constraints, {{i, cosine}}, -config.gamma * left * units) ||
            !appendHalfspace(constraints, {{i, -cosine}}, -config.gamma * right * units) ||
            !appendHalfspace(constraints, {{i, sine}}, -config.gamma * bottom * units) ||
            !appendHalfspace(constraints, {{i, -sine}}, -config.gamma * top * units)) {
            return false;
        }
    }

    const double minimumDistanceSquared =
        config.minimumDistance * config.minimumDistance;
    for (std::size_t i = 0; i < robots.size(); ++i) {
        for (std::size_t j = i + 1; j < robots.size(); ++j) {
            const double dx = robots[i].pose.x - robots[j].pose.x;
            const double dy = robots[i].pose.y - robots[j].pose.y;
            const double distanceSquared = dx * dx + dy * dy;
            const double h = distanceSquared - minimumDistanceSquared;
            const double firstCoefficient = 2.0 *
                (dx * std::cos(robots[i].pose.yaw) +
                 dy * std::sin(robots[i].pose.yaw));
            const double secondCoefficient = -2.0 *
                (dx * std::cos(robots[j].pose.yaw) +
                 dy * std::sin(robots[j].pose.yaw));
            if (!appendHalfspace(
                    constraints,
                    {{i, firstCoefficient}, {j, secondCoefficient}},
                    -config.gamma * h * units)) {
                return false;
            }
        }
    }
    return true;
}

bool appendStrictHalfspace(
    std::vector<StrictHalfspace>& constraints,
    std::vector<StrictConstraintTerm> terms,
    const ExtendedValue& lowerBound)
{
    if (!finiteExtended(lowerBound)) return false;
    for (const StrictConstraintTerm& term : terms) {
        if (!finiteExtended(term.coefficient)) return false;
    }
    constraints.push_back({std::move(terms), lowerBound});
    return true;
}

bool buildStrictConstraints(const std::vector<CbfRobotState>& robots,
                            const std::vector<double>& differentials,
                            const ZooidCbfConfig& config,
                            std::vector<StrictHalfspace>& constraints)
{
    constraints.clear();
    const ExtendedValue zero = {0.0, 0.0};
    const ExtendedValue one = {1.0, 0.0};
    const ExtendedValue minusOne = {-1.0, 0.0};
    const ExtendedValue wheelLimit = {
        static_cast<double>(config.maximumWheelCommand), 0.0};
    const ExtendedValue gammaUnits = multiplyExtended(
        {config.gamma, 0.0},
        {config.commandUnitsPerMetrePerSecond, 0.0});
    const ExtendedValue maximumX =
        exactDifference(config.fieldWidth, config.safeRadius);
    const ExtendedValue maximumY =
        exactDifference(config.fieldHeight, config.safeRadius);
    for (std::size_t i = 0; i < robots.size(); ++i) {
        const ExtendedValue differential = {differentials[i], 0.0};
        const ExtendedValue cosine = {
            std::cos(robots[i].pose.yaw), 0.0};
        const ExtendedValue sine = {
            std::sin(robots[i].pose.yaw), 0.0};
        const ExtendedValue left = exactDifference(
            robots[i].pose.x, config.safeRadius);
        const ExtendedValue right = subtractExtended(
            maximumX, {robots[i].pose.x, 0.0});
        const ExtendedValue bottom = exactDifference(
            robots[i].pose.y, config.safeRadius);
        const ExtendedValue top = subtractExtended(
            maximumY, {robots[i].pose.y, 0.0});

        if (!appendStrictHalfspace(constraints, {{i, one}}, zero) ||
            !appendStrictHalfspace(
                constraints, {{i, one}},
                subtractExtended(differential, wheelLimit)) ||
            !appendStrictHalfspace(
                constraints, {{i, minusOne}},
                negateExtended(addExtended(wheelLimit, differential))) ||
            !appendStrictHalfspace(
                constraints, {{i, one}},
                negateExtended(addExtended(wheelLimit, differential))) ||
            !appendStrictHalfspace(
                constraints, {{i, minusOne}},
                subtractExtended(differential, wheelLimit)) ||
            !appendStrictHalfspace(
                constraints, {{i, cosine}},
                negateExtended(multiplyExtended(gammaUnits, left))) ||
            !appendStrictHalfspace(
                constraints, {{i, negateExtended(cosine)}},
                negateExtended(multiplyExtended(gammaUnits, right))) ||
            !appendStrictHalfspace(
                constraints, {{i, sine}},
                negateExtended(multiplyExtended(gammaUnits, bottom))) ||
            !appendStrictHalfspace(
                constraints, {{i, negateExtended(sine)}},
                negateExtended(multiplyExtended(gammaUnits, top)))) {
            return false;
        }
    }

    const ExtendedValue minimumDistanceSquared = squareExtended(
        {config.minimumDistance, 0.0});
    for (std::size_t i = 0; i < robots.size(); ++i) {
        for (std::size_t j = i + 1; j < robots.size(); ++j) {
            const ExtendedValue dx = exactDifference(
                robots[i].pose.x, robots[j].pose.x);
            const ExtendedValue dy = exactDifference(
                robots[i].pose.y, robots[j].pose.y);
            const ExtendedValue h = subtractExtended(
                addExtended(squareExtended(dx), squareExtended(dy)),
                minimumDistanceSquared);
            const ExtendedValue firstDirection = addExtended(
                multiplyExtended(
                    dx, {std::cos(robots[i].pose.yaw), 0.0}),
                multiplyExtended(
                    dy, {std::sin(robots[i].pose.yaw), 0.0}));
            const ExtendedValue secondDirection = addExtended(
                multiplyExtended(
                    dx, {std::cos(robots[j].pose.yaw), 0.0}),
                multiplyExtended(
                    dy, {std::sin(robots[j].pose.yaw), 0.0}));
            if (!appendStrictHalfspace(
                    constraints,
                    {{i, multiplyExtended({2.0, 0.0}, firstDirection)},
                     {j, negateExtended(multiplyExtended(
                         {2.0, 0.0}, secondDirection))}},
                    negateExtended(multiplyExtended(gammaUnits, h)))) {
                return false;
            }
        }
    }
    return true;
}

double constraintValue(const Halfspace& constraint,
                       const std::vector<double>& values)
{
    double value = 0.0;
    for (const ConstraintTerm& term : constraint.terms)
        value += term.coefficient * values[term.index];
    return value;
}

bool projectionConstraintsSatisfied(
    const std::vector<double>& values,
    const std::vector<Halfspace>& constraints)
{
    for (const Halfspace& constraint : constraints) {
        const double value = constraintValue(constraint, values);
        if (!std::isfinite(value) || value < constraint.lowerBound) {
            return false;
        }
    }
    return true;
}

bool strictConstraintsSatisfied(
    const std::vector<double>& values,
    const std::vector<StrictHalfspace>& constraints)
{
    for (const StrictHalfspace& constraint : constraints) {
        ExtendedValue value = {0.0, 0.0};
        for (const StrictConstraintTerm& term : constraint.terms) {
            value = addExtended(value, multiplyExtended(
                term.coefficient, {values[term.index], 0.0}));
        }
        if (!finiteExtended(value) ||
            extendedLess(value, constraint.lowerBound)) {
            return false;
        }
    }
    return true;
}

bool projectDykstra(const std::vector<double>& initial,
                    const std::vector<Halfspace>& constraints,
                    const ZooidCbfConfig& config,
                    std::vector<double>& projected)
{
    projected = initial;
    std::vector<double> corrections(constraints.size(), 0.0);
    const double tolerance = effectiveTolerance(config);
    for (unsigned int iteration = 0;
         iteration < config.maximumIterations;
         ++iteration) {
        const std::vector<double> cycleStart = projected;
        for (std::size_t j = 0; j < constraints.size(); ++j) {
            const Halfspace& constraint = constraints[j];
            for (const ConstraintTerm& term : constraint.terms)
                projected[term.index] += corrections[j] * term.coefficient;

            const double value = constraintValue(constraint, projected);
            const double violation = constraint.lowerBound - value;
            if (!std::isfinite(value) || !std::isfinite(violation)) return false;
            if (violation > 0.0) {
                if (constraint.normSquared == 0.0) return false;
                const double amount = violation / constraint.normSquared;
                if (!std::isfinite(amount)) return false;
                for (const ConstraintTerm& term : constraint.terms)
                    projected[term.index] += amount * term.coefficient;
                corrections[j] = -amount;
            } else {
                corrections[j] = 0.0;
            }
        }

        double maximumChange = 0.0;
        for (std::size_t i = 0; i < projected.size(); ++i) {
            if (!std::isfinite(projected[i])) return false;
            maximumChange = std::max(
                maximumChange, std::abs(projected[i] - cycleStart[i]));
        }
        if (maximumChange <= tolerance &&
            projectionConstraintsSatisfied(projected, constraints)) {
            return true;
        }
    }
    return false;
}

void commandModes(const std::vector<CbfRobotState>& robots,
                  const std::map<unsigned int, WheelCommand>& commands,
                  std::vector<double>& commonModes,
                  std::vector<double>& differentials)
{
    commonModes.clear();
    differentials.clear();
    for (const CbfRobotState& robot : robots) {
        const WheelCommand command = commands.at(robot.id);
        commonModes.push_back(
            (static_cast<double>(command.left) + command.right) / 2.0);
        differentials.push_back(
            (static_cast<double>(command.right) - command.left) / 2.0);
    }
}

bool reconstructCommands(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& nominal,
    const std::vector<double>& nominalCommonModes,
    const std::vector<double>& projected,
    std::map<unsigned int, WheelCommand>& commands)
{
    commands.clear();
    const double longMinimum =
        static_cast<double>(std::numeric_limits<long>::min()) + 32768.0;
    const double longMaximum =
        static_cast<double>(std::numeric_limits<long>::max()) - 32768.0;
    for (std::size_t i = 0; i < robots.size(); ++i) {
        const double commonShift = projected[i] - nominalCommonModes[i];
        if (!std::isfinite(commonShift) ||
            commonShift < longMinimum || commonShift > longMaximum) {
            return false;
        }
        const long shift = std::lround(commonShift);
        const WheelCommand original = nominal.at(robots[i].id);
        const long left = static_cast<long>(original.left) + shift;
        const long right = static_cast<long>(original.right) + shift;
        if (left < std::numeric_limits<int16_t>::min() ||
            left > std::numeric_limits<int16_t>::max() ||
            right < std::numeric_limits<int16_t>::min() ||
            right > std::numeric_limits<int16_t>::max()) {
            return false;
        }
        commands[robots[i].id] = {
            static_cast<int16_t>(left), static_cast<int16_t>(right)};
    }
    return true;
}

struct DiscreteNode
{
    std::vector<std::size_t> ranks;
    std::vector<double> commonModes;
    double projectedDistanceSquared = 0.0;
    std::size_t minimumIncrementIndex = 0;
};

struct DiscreteNodeIsWorse
{
    bool operator()(const DiscreteNode& first,
                    const DiscreteNode& second) const
    {
        if (first.projectedDistanceSquared !=
            second.projectedDistanceSquared) {
            return first.projectedDistanceSquared >
                second.projectedDistanceSquared;
        }
        return std::lexicographical_compare(
            second.commonModes.begin(), second.commonModes.end(),
            first.commonModes.begin(), first.commonModes.end());
    }
};

double projectedDistanceSquared(const std::vector<double>& commonModes,
                                const std::vector<double>& projected)
{
    double distanceSquared = 0.0;
    for (std::size_t i = 0; i < commonModes.size(); ++i) {
        const double difference = commonModes[i] - projected[i];
        distanceSquared += difference * difference;
    }
    return distanceSquared;
}

bool findDiscreteCommands(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& nominal,
    const std::vector<double>& nominalCommonModes,
    const std::vector<double>& projected,
    const std::vector<StrictHalfspace>& constraints,
    const ZooidCbfConfig& config,
    std::map<unsigned int, WheelCommand>& commands)
{
    std::vector<std::vector<double>> candidates(robots.size());
    std::vector<double> targets(robots.size());
    std::vector<double> minimumCommonFallback;
    minimumCommonFallback.reserve(robots.size());
    for (std::size_t i = 0; i < robots.size(); ++i) {
        const double nominalCommon = nominalCommonModes[i];
        const WheelCommand original = nominal.at(robots[i].id);
        const long wheelLimit = config.maximumWheelCommand;
        const long minimumShift = std::max({
            -wheelLimit - static_cast<long>(original.left),
            -wheelLimit - static_cast<long>(original.right),
            static_cast<long>(std::ceil(-nominalCommon))});
        const long maximumShift = std::min(
            wheelLimit - static_cast<long>(original.left),
            wheelLimit - static_cast<long>(original.right));
        if (minimumShift > maximumShift) return false;
        minimumCommonFallback.push_back(nominalCommon + minimumShift);
        for (long shift = minimumShift; shift <= maximumShift; ++shift) {
            candidates[i].push_back(nominalCommon + shift);
        }

        targets[i] = i < projected.size() && std::isfinite(projected[i])
            ? projected[i]
            : nominalCommon;
        std::sort(candidates[i].begin(), candidates[i].end(),
                  [target = targets[i]](double first, double second) {
                      const double firstDifference = first - target;
                      const double secondDifference = second - target;
                      const double firstCost = firstDifference * firstDifference;
                      const double secondCost = secondDifference * secondDifference;
                      return firstCost != secondCost
                          ? firstCost < secondCost
                          : first < second;
                  });
    }
    const bool fallbackSafe = strictConstraintsSatisfied(
        minimumCommonFallback, constraints);

    DiscreteNode initial;
    initial.ranks.assign(robots.size(), 0);
    for (const std::vector<double>& robotCandidates : candidates)
        initial.commonModes.push_back(robotCandidates.front());
    initial.projectedDistanceSquared =
        projectedDistanceSquared(initial.commonModes, targets);
    if (!std::isfinite(initial.projectedDistanceSquared)) {
        return fallbackSafe && reconstructCommands(
            robots, nominal, nominalCommonModes,
            minimumCommonFallback, commands);
    }

    std::priority_queue<DiscreteNode,
                        std::vector<DiscreteNode>,
                        DiscreteNodeIsWorse> frontier;
    frontier.push(std::move(initial));
    uint64_t createdNodes = 1;
    while (!frontier.empty()) {
        DiscreteNode node = frontier.top();
        frontier.pop();
        if (strictConstraintsSatisfied(node.commonModes, constraints)) {
            return reconstructCommands(
                robots, nominal, nominalCommonModes,
                node.commonModes, commands);
        }
        if (createdNodes >= config.maximumDiscreteSearchNodes) continue;

        for (std::size_t i = node.minimumIncrementIndex;
             i < robots.size(); ++i) {
            if (createdNodes >= config.maximumDiscreteSearchNodes) break;
            if (node.ranks[i] + 1 >= candidates[i].size()) continue;
            DiscreteNode next = node;
            ++next.ranks[i];
            next.commonModes[i] = candidates[i][next.ranks[i]];
            next.projectedDistanceSquared =
                projectedDistanceSquared(next.commonModes, targets);
            next.minimumIncrementIndex = i;
            if (std::isfinite(next.projectedDistanceSquared)) {
                frontier.push(std::move(next));
                ++createdNodes;
            }
        }
    }
    return fallbackSafe && reconstructCommands(
        robots, nominal, nominalCommonModes,
        minimumCommonFallback, commands);
}

bool sameCommands(const std::map<unsigned int, WheelCommand>& first,
                  const std::map<unsigned int, WheelCommand>& second)
{
    if (first.size() != second.size()) return false;
    for (const auto& entry : first) {
        const auto found = second.find(entry.first);
        if (found == second.end() ||
            found->second.left != entry.second.left ||
            found->second.right != entry.second.right) {
            return false;
        }
    }
    return true;
}

} // namespace

bool zooidCbfConstraintsSatisfied(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& commands,
    const ZooidCbfConfig& config)
{
    std::vector<CbfRobotState> sorted;
    if (validateStructure(robots, commands, config, sorted) != InputValidity::Valid ||
        validateGeometry(sorted, config) != InputValidity::Valid) {
        return false;
    }

    std::vector<double> commonModes;
    std::vector<double> differentials;
    commandModes(sorted, commands, commonModes, differentials);
    std::vector<StrictHalfspace> constraints;
    return buildStrictConstraints(
            sorted, differentials, config, constraints) &&
        strictConstraintsSatisfied(commonModes, constraints);
}

ZooidCbfResult applyZooidCbf(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& nominal,
    uint64_t nowMs,
    const ZooidCbfConfig& config)
{
    std::vector<CbfRobotState> sorted;
    if (validateStructure(robots, nominal, config, sorted) != InputValidity::Valid ||
        !validTiming(sorted, nowMs, config)) {
        return zeroResult(robots, ZooidCbfStatus::InvalidInput);
    }
    const InputValidity geometry = validateGeometry(sorted, config);
    if (geometry == InputValidity::Invalid)
        return zeroResult(robots, ZooidCbfStatus::InvalidInput);
    if (geometry == InputValidity::Unsafe)
        return zeroResult(robots, ZooidCbfStatus::UnsafeState);

    std::vector<double> nominalCommonModes;
    std::vector<double> differentials;
    commandModes(sorted, nominal, nominalCommonModes, differentials);
    std::vector<Halfspace> constraints;
    std::vector<StrictHalfspace> strictConstraints;
    if (!buildConstraints(sorted, differentials, config, constraints) ||
        !buildStrictConstraints(
            sorted, differentials, config, strictConstraints)) {
        return zeroResult(robots, ZooidCbfStatus::SolverFailure);
    }

    if (strictConstraintsSatisfied(
            nominalCommonModes, strictConstraints)) {
        ZooidCbfResult result;
        result.status = ZooidCbfStatus::Safe;
        result.commands = nominal;
        return result;
    }

    std::vector<double> projected;
    std::map<unsigned int, WheelCommand> commands;
    projectDykstra(nominalCommonModes, constraints, config, projected);
    if (!findDiscreteCommands(
            sorted, nominal, nominalCommonModes, projected,
            strictConstraints, config, commands)) {
        return zeroResult(robots, ZooidCbfStatus::SolverFailure);
    }

    ZooidCbfResult result;
    result.status = sameCommands(commands, nominal)
        ? ZooidCbfStatus::Safe
        : ZooidCbfStatus::Intervened;
    result.commands = std::move(commands);
    return result;
}
