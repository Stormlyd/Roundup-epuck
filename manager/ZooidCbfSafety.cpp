#include "ZooidCbfSafety.h"

#include <algorithm>
#include <cmath>
#include <limits>
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
        config.maximumIterations > 0 && config.tolerance >= 0.0;
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
    for (const CbfRobotState& robot : robots) {
        if (robot.pose.x < config.safeRadius ||
            robot.pose.x > config.fieldWidth - config.safeRadius ||
            robot.pose.y < config.safeRadius ||
            robot.pose.y > config.fieldHeight - config.safeRadius) {
            return InputValidity::Unsafe;
        }
    }

    const double minimumDistanceSquared =
        config.minimumDistance * config.minimumDistance;
    if (!std::isfinite(minimumDistanceSquared)) return InputValidity::Invalid;
    for (std::size_t i = 0; i < robots.size(); ++i) {
        for (std::size_t j = i + 1; j < robots.size(); ++j) {
            const double dx = robots[i].pose.x - robots[j].pose.x;
            const double dy = robots[i].pose.y - robots[j].pose.y;
            const double distanceSquared = dx * dx + dy * dy;
            if (!std::isfinite(distanceSquared)) return InputValidity::Invalid;
            if (distanceSquared < minimumDistanceSquared)
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

double constraintValue(const Halfspace& constraint,
                       const std::vector<double>& values)
{
    double value = 0.0;
    for (const ConstraintTerm& term : constraint.terms)
        value += term.coefficient * values[term.index];
    return value;
}

bool constraintsSatisfied(const std::vector<double>& values,
                          const std::vector<Halfspace>& constraints,
                          double tolerance)
{
    for (const Halfspace& constraint : constraints) {
        const double value = constraintValue(constraint, values);
        if (!std::isfinite(value) ||
            constraint.lowerBound - value > tolerance) {
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
            constraintsSatisfied(projected, constraints, tolerance)) {
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

void searchDiscreteCombinations(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& nominal,
    const std::vector<double>& nominalCommonModes,
    const std::vector<std::vector<double>>& candidates,
    const ZooidCbfConfig& config,
    std::size_t index,
    double cost,
    std::vector<double>& current,
    bool& found,
    double& bestCost,
    std::vector<double>& bestCommonModes,
    std::map<unsigned int, WheelCommand>& bestCommands)
{
    if (found && cost > bestCost) return;
    if (index == robots.size()) {
        std::map<unsigned int, WheelCommand> commands;
        if (!reconstructCommands(
                robots, nominal, nominalCommonModes, current, commands) ||
            !zooidCbfConstraintsSatisfied(robots, commands, config)) {
            return;
        }
        if (!found || cost < bestCost ||
            (cost == bestCost && std::lexicographical_compare(
                current.begin(), current.end(),
                bestCommonModes.begin(), bestCommonModes.end()))) {
            found = true;
            bestCost = cost;
            bestCommonModes = current;
            bestCommands = std::move(commands);
        }
        return;
    }

    for (double candidate : candidates[index]) {
        const double difference = candidate - nominalCommonModes[index];
        const double nextCost = cost + difference * difference;
        if (!std::isfinite(nextCost)) continue;
        current[index] = candidate;
        searchDiscreteCombinations(
            robots, nominal, nominalCommonModes, candidates, config,
            index + 1, nextCost, current, found, bestCost,
            bestCommonModes, bestCommands);
    }
}

bool findDiscreteCommands(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& nominal,
    const std::vector<double>& nominalCommonModes,
    const std::vector<double>& projected,
    const ZooidCbfConfig& config,
    std::map<unsigned int, WheelCommand>& commands)
{
    std::vector<std::vector<double>> candidates(robots.size());
    for (std::size_t i = 0; i < robots.size(); ++i) {
        const double nominalCommon = nominalCommonModes[i];
        const auto addShift = [&](double shift) {
            const double candidate = nominalCommon + shift;
            if (std::isfinite(candidate) &&
                std::find(candidates[i].begin(), candidates[i].end(), candidate) ==
                    candidates[i].end()) {
                candidates[i].push_back(candidate);
            }
        };
        if (i < projected.size() && std::isfinite(projected[i])) {
            const double projectedShift = projected[i] - nominalCommon;
            addShift(std::floor(projectedShift));
            addShift(std::ceil(projectedShift));
        }
        addShift(-1.0);
        addShift(0.0);
        addShift(1.0);
        addShift(std::floor(-nominalCommon));
        addShift(std::ceil(-nominalCommon));
        std::sort(candidates[i].begin(), candidates[i].end(),
                  [nominalCommon](double first, double second) {
                      const double firstDifference = first - nominalCommon;
                      const double secondDifference = second - nominalCommon;
                      const double firstCost = firstDifference * firstDifference;
                      const double secondCost = secondDifference * secondDifference;
                      return firstCost != secondCost
                          ? firstCost < secondCost
                          : first < second;
                  });
    }

    bool found = false;
    double bestCost = std::numeric_limits<double>::infinity();
    std::vector<double> current(robots.size(), 0.0);
    std::vector<double> bestCommonModes;
    searchDiscreteCombinations(
        robots, nominal, nominalCommonModes, candidates, config,
        0, 0.0, current, found, bestCost, bestCommonModes, commands);
    return found;
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
    std::vector<Halfspace> constraints;
    return buildConstraints(sorted, differentials, config, constraints) &&
        constraintsSatisfied(
            commonModes, constraints, effectiveTolerance(config));
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
    if (!buildConstraints(sorted, differentials, config, constraints))
        return zeroResult(robots, ZooidCbfStatus::SolverFailure);

    if (constraintsSatisfied(
            nominalCommonModes, constraints, effectiveTolerance(config))) {
        ZooidCbfResult result;
        result.status = ZooidCbfStatus::Safe;
        result.commands = nominal;
        return result;
    }

    std::vector<double> projected;
    std::map<unsigned int, WheelCommand> commands;
    projectDykstra(nominalCommonModes, constraints, config, projected);
    if (!findDiscreteCommands(
            sorted, nominal, nominalCommonModes, projected, config, commands)) {
        return zeroResult(robots, ZooidCbfStatus::SolverFailure);
    }

    ZooidCbfResult result;
    result.status = sameCommands(commands, nominal)
        ? ZooidCbfStatus::Safe
        : ZooidCbfStatus::Intervened;
    result.commands = std::move(commands);
    return result;
}
