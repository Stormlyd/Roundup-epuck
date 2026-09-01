#include "ZooidTestMode.h"

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
           nowMs >= robot.feedbackMs &&
           nowMs - robot.feedbackMs < PursuitProfile::FeedbackTimeoutMs;
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
    estimatedTargetVx_ = 0.0;
    estimatedTargetVy_ = 0.0;
    lastAcceptedFeedbackMs_ = {{0, 0, 0, 0}};
    for (const auto& robot : robots)
    {
        if (robot.id == roles_.targetId)
            lastAcceptedFeedbackMs_[0] = robot.feedbackMs;
        for (std::size_t i = 0; i < roles_.pursuerIds.size(); ++i)
            if (robot.id == roles_.pursuerIds[i])
                lastAcceptedFeedbackMs_[i + 1] = robot.feedbackMs;
    }
    observationSequence_ = 0;
    lastOutput_ = {};
    haveLastOutput_ = false;
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
        if (nowMs < found->second.feedbackMs ||
            nowMs - found->second.feedbackMs >= PursuitProfile::FeedbackTimeoutMs)
            return faultOutput(PursuitFault::FeedbackStale, robots);
    }

    PursuitWorldState world;
    world.target = byId[roles_.targetId];
    for (std::size_t i = 0; i < 3; ++i) world.pursuers[i] = byId[roles_.pursuerIds[i]];
    world.fieldWidth = fieldWidth;
    world.fieldHeight = fieldHeight;
    const std::array<uint64_t, 4> feedbackStamps{{
        world.target.feedbackMs,
        world.pursuers[0].feedbackMs,
        world.pursuers[1].feedbackMs,
        world.pursuers[2].feedbackMs
    }};
    bool completeFreshObservation = true;
    for (std::size_t i = 0; i < feedbackStamps.size(); ++i)
        completeFreshObservation = completeFreshObservation &&
            feedbackStamps[i] > lastAcceptedFeedbackMs_[i];
    const uint64_t oldestFeedback = *std::min_element(
        feedbackStamps.begin(), feedbackStamps.end());
    completeFreshObservation = completeFreshObservation &&
        nowMs - oldestFeedback < PursuitProfile::CommandHoldTimeoutMs;
    (void)sequence;

    if (havePreviousTarget_ && world.target.feedbackMs > previousTargetMs_)
    {
        const double elapsed = static_cast<double>(
            world.target.feedbackMs - previousTargetMs_) / 1000.0;
        double measuredVx = (world.target.pose.x - previousTargetPose_.x) / elapsed;
        double measuredVy = (world.target.pose.y - previousTargetPose_.y) / elapsed;
        const double measuredSpeed = std::hypot(measuredVx, measuredVy);
        if (measuredSpeed > PursuitProfile::TargetVelocityEstimateLimit)
        {
            measuredVx *= PursuitProfile::TargetVelocityEstimateLimit / measuredSpeed;
            measuredVy *= PursuitProfile::TargetVelocityEstimateLimit / measuredSpeed;
        }
        const double filterTimeConstant = 0.20;
        const double alpha = elapsed / (filterTimeConstant + elapsed);
        estimatedTargetVx_ += alpha * (measuredVx - estimatedTargetVx_);
        estimatedTargetVy_ += alpha * (measuredVy - estimatedTargetVy_);
        previousTargetPose_ = world.target.pose;
        previousTargetMs_ = world.target.feedbackMs;
    }
    else if (!havePreviousTarget_ || world.target.feedbackMs < previousTargetMs_)
    {
        previousTargetPose_ = world.target.pose;
        previousTargetMs_ = world.target.feedbackMs;
        estimatedTargetVx_ = 0.0;
        estimatedTargetVy_ = 0.0;
        havePreviousTarget_ = true;
    }
    world.target.vx = estimatedTargetVx_;
    world.target.vy = estimatedTargetVy_;

    if (!completeFreshObservation)
    {
        PursuitControlOutput output = haveLastOutput_ ? lastOutput_ : PursuitControlOutput{};
        output.phase = machine_.phase();
        output.fault = PursuitFault::None;
        if (nowMs - oldestFeedback >= PursuitProfile::CommandHoldTimeoutMs)
        {
            smoother_.clear();
            output.commands.clear();
            for (const auto& robot : robots)
                if (robot.connected && robot.activated)
                    output.commands[robot.id] = {0, 0};
            for (unsigned int id : roles_.participantIds())
                output.commands[id] = {0, 0};
            lastOutput_ = output;
            haveLastOutput_ = true;
        }
        else
        {
            const std::vector<unsigned int> participants = roles_.participantIds();
            for (const auto& robot : robots)
                if (robot.connected && robot.activated &&
                    !containsId(participants, robot.id))
                    output.commands[robot.id] = {0, 0};
        }
        return output;
    }

    lastAcceptedFeedbackMs_ = feedbackStamps;
    world.stampMs = *std::min_element(feedbackStamps.begin(), feedbackStamps.end());
    world.sequence = ++observationSequence_;

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

    status_.phase = phaseResult.phase;
    status_.fault = PursuitFault::None;
    status_.roles = roles_;
    status_.event = phaseResult.event;
    status_.captureProgress = phaseResult.captureProgress;
    for (std::size_t i = 0; i < 3; ++i)
        status_.targetDistances[i] = distanceBetween(world.target.pose, world.pursuers[i].pose);
    lastOutput_ = output;
    haveLastOutput_ = true;
    if (phaseResult.phase == PursuitPhase::Captured) running_ = false;
    return output;
}

PursuitRoleMap ZooidTestMode::roleMap() const { return roles_; }
PursuitStatusSnapshot ZooidTestMode::statusSnapshot() const { return status_; }
