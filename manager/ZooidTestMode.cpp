#include "ZooidTestMode.h"

#include "ZooidCbfSafety.h"
#include "ZooidPursuitGeometry.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace
{
bool finiteRobot(const PursuitRobotState& robot)
{
    return std::isfinite(robot.pose.x) && std::isfinite(robot.pose.y) &&
           std::isfinite(robot.pose.yaw);
}

bool freshRobot(const PursuitRobotState& robot, uint64_t nowMs)
{
    return robot.connected && robot.activated && finiteRobot(robot) &&
           nowMs >= robot.feedbackMs && nowMs - robot.feedbackMs < 500;
}

bool containsId(const std::vector<unsigned int>& ids, unsigned int id)
{
    return std::find(ids.begin(), ids.end(), id) != ids.end();
}
}

bool ZooidTestMode::start(const std::vector<PursuitRobotState>& robots, uint64_t nowMs)
{
    if (running_) return false;
    std::vector<unsigned int> freshIds;
    for (const auto& robot : robots)
        if (freshRobot(robot, nowMs)) freshIds.push_back(robot.id);

    PursuitRoleMap assigned;
    if (!assignPursuitRoles(freshIds, assigned))
    {
        status_ = {};
        status_.fault = PursuitFault::NotEnoughRobots;
        return false;
    }

    roles_ = assigned;
    status_ = {};
    status_.roles = roles_;
    status_.phase = PursuitPhase::Pursuit;
    machine_.reset();
    slots_.clear();
    smoother_.clear();
    machine_.start();
    previousTargetMs_ = 0;
    havePreviousTarget_ = false;
    running_ = true;
    return true;
}

void ZooidTestMode::stop()
{
    running_ = false;
    machine_.stop();
    slots_.clear();
    smoother_.clear();
    status_.phase = PursuitPhase::Idle;
    status_.fault = PursuitFault::ManualStop;
    status_.event = "manual_stop";
}

bool ZooidTestMode::isRunning() const { return running_; }

PursuitControlOutput ZooidTestMode::stoppedOutput(
    const std::vector<PursuitRobotState>& robots) const
{
    PursuitControlOutput output;
    output.phase = status_.phase;
    output.fault = status_.fault;
    output.event = status_.event;
    output.captureProgress = status_.captureProgress;
    for (const auto& robot : robots)
        if (robot.connected && robot.activated) output.commands[robot.id] = {0, 0};
    for (unsigned int id : roles_.participantIds()) output.commands[id] = {0, 0};
    return output;
}

PursuitControlOutput ZooidTestMode::faultOutput(
    PursuitFault fault,
    const std::vector<PursuitRobotState>& robots)
{
    running_ = false;
    status_.fault = fault;
    status_.event = "safety_stop";
    smoother_.clear();
    return stoppedOutput(robots);
}

PursuitControlOutput ZooidTestMode::update(
    const std::vector<PursuitRobotState>& robots,
    uint64_t nowMs,
    double fieldWidth,
    double fieldHeight,
    uint64_t sequence)
{
    if (!running_) return stoppedOutput(robots);

    std::map<unsigned int, PursuitRobotState> byId;
    for (const auto& robot : robots) byId[robot.id] = robot;
    for (unsigned int id : roles_.participantIds())
    {
        const auto found = byId.find(id);
        if (found == byId.end() || !found->second.connected || !found->second.activated)
            return faultOutput(PursuitFault::ParticipantMissing, robots);
        if (!finiteRobot(found->second))
            return faultOutput(PursuitFault::InvalidFeedback, robots);
        if (nowMs < found->second.feedbackMs || nowMs - found->second.feedbackMs >= 500)
            return faultOutput(PursuitFault::FeedbackStale, robots);
    }

    PursuitWorldState world;
    world.target = byId[roles_.targetId];
    for (std::size_t i = 0; i < 3; ++i) world.pursuers[i] = byId[roles_.pursuerIds[i]];
    world.fieldWidth = fieldWidth;
    world.fieldHeight = fieldHeight;
    world.stampMs = nowMs;
    world.sequence = sequence;

    if (havePreviousTarget_ && nowMs > previousTargetMs_)
    {
        const double elapsed = static_cast<double>(nowMs - previousTargetMs_) / 1000.0;
        world.target.vx = std::max(-0.03, std::min(0.03,
            (world.target.pose.x - previousTargetPose_.x) / elapsed));
        world.target.vy = std::max(-0.03, std::min(0.03,
            (world.target.pose.y - previousTargetPose_.y) / elapsed));
    }
    previousTargetPose_ = world.target.pose;
    previousTargetMs_ = nowMs;
    havePreviousTarget_ = true;

    const PursuitPhaseResult phaseResult = machine_.update(world);
    PursuitControlOutput output = computePursuitCommands(
        world, roles_, phaseResult.phase, slots_, smoother_);
    output.event = phaseResult.event;
    output.captureProgress = phaseResult.captureProgress;
    if (output.fault != PursuitFault::None)
        return faultOutput(output.fault, robots);

    const std::vector<unsigned int> participants = roles_.participantIds();
    for (const auto& robot : robots)
        if (robot.connected && robot.activated && !containsId(participants, robot.id))
            output.commands[robot.id] = {0, 0};

    std::vector<CbfRobotState> cbfRobots;
    for (const auto& robot : robots)
        if (robot.connected && robot.activated)
            cbfRobots.push_back({robot.id, robot.pose, robot.feedbackMs});
    const ZooidCbfConfig cbfConfig;
    const ZooidCbfResult cbfResult = applyZooidCbf(
        cbfRobots, output.commands, nowMs, cbfConfig);
    if (cbfResult.status != ZooidCbfStatus::Safe &&
        cbfResult.status != ZooidCbfStatus::Intervened)
        return faultOutput(PursuitFault::SafetyViolation, robots);
    output.commands = cbfResult.commands;

    status_.phase = phaseResult.phase;
    status_.fault = PursuitFault::None;
    status_.roles = roles_;
    status_.event = phaseResult.event;
    status_.captureProgress = phaseResult.captureProgress;
    for (std::size_t i = 0; i < 3; ++i)
        status_.targetDistances[i] = distanceBetween(world.target.pose, world.pursuers[i].pose);
    if (phaseResult.phase == PursuitPhase::Captured) running_ = false;
    return output;
}

PursuitRoleMap ZooidTestMode::roleMap() const { return roles_; }
PursuitStatusSnapshot ZooidTestMode::statusSnapshot() const { return status_; }
