#include "ZooidPursuitStateMachine.h"

#include "ZooidPursuitGeometry.h"

#include <algorithm>

PursuitStateMachine::PursuitStateMachine(const PursuitConfig& config)
    : config_(config)
{
}

bool PursuitStateMachine::start()
{
    if (phase_ != PursuitPhase::Idle)
        return false;
    transition(PursuitPhase::Pursuit);
    haveObservation_ = false;
    return true;
}

void PursuitStateMachine::stop()
{
    reset();
}

void PursuitStateMachine::reset()
{
    phase_ = PursuitPhase::Idle;
    phaseTicks_ = readyTicks_ = captureTicks_ = breakTicks_ = 0;
    lastSequence_ = lastStampMs_ = 0;
    haveObservation_ = false;
    captureAcquired_ = false;
}

PursuitPhase PursuitStateMachine::phase() const
{
    return phase_;
}

bool PursuitStateMachine::acceptFresh(const PursuitWorldState& world)
{
    if (haveObservation_ &&
        (world.sequence == lastSequence_ || world.stampMs == lastStampMs_))
        return false;
    if (haveObservation_ && world.sequence < lastSequence_ && world.stampMs <= lastStampMs_)
    {
        phaseTicks_ = readyTicks_ = captureTicks_ = breakTicks_ = 0;
        captureAcquired_ = false;
    }
    lastSequence_ = world.sequence;
    lastStampMs_ = world.stampMs;
    haveObservation_ = true;
    return true;
}

void PursuitStateMachine::transition(PursuitPhase phase)
{
    phase_ = phase;
    phaseTicks_ = readyTicks_ = captureTicks_ = breakTicks_ = 0;
    captureAcquired_ = false;
}

bool PursuitStateMachine::ringFeasible(const PursuitWorldState& world, double radius) const
{
    for (int sample = 0; sample < 24; ++sample)
    {
        const double heading = sample * 6.28318530717958647692 / 24.0;
        if (ringInsideBounds(makeTriangularRing(world.target.pose, radius, heading),
                             world.fieldWidth, world.fieldHeight, 0.057))
            return true;
    }
    return false;
}

PursuitPhaseResult PursuitStateMachine::update(const PursuitWorldState& world)
{
    PursuitPhaseResult result;
    result.phase = phase_;
    if (phase_ == PursuitPhase::Idle || phase_ == PursuitPhase::Captured || !acceptFresh(world))
    {
        result.captureProgress = std::min(1.0,
            static_cast<double>(captureTicks_) / std::max(1, config_.captureStableTicks));
        return result;
    }

    ++phaseTicks_;
    if (phase_ == PursuitPhase::Pursuit)
    {
        bool ready = phaseTicks_ >= config_.pursuitMinTicks &&
                     ringFeasible(world, config_.surroundRadius);
        for (const auto& pursuer : world.pursuers)
            ready = ready && distanceBetween(world.target.pose, pursuer.pose) <= config_.pursuitRadius;
        readyTicks_ = ready ? readyTicks_ + 1 : 0;
        if (readyTicks_ >= config_.transitionReadyTicks)
        {
            transition(PursuitPhase::Surround);
            result.phase = phase_;
            result.event = "surround_started";
        }
        return result;
    }

    std::array<PursuitPose, 3> poses;
    for (std::size_t i = 0; i < 3; ++i) poses[i] = world.pursuers[i].pose;
    if (phase_ == PursuitPhase::Surround)
    {
        bool broken = !ringFeasible(world, config_.surroundRadius);
        for (const auto& pursuer : world.pursuers)
            broken = broken || distanceBetween(world.target.pose, pursuer.pose) > config_.pursuitBreakRadius;
        breakTicks_ = broken ? breakTicks_ + 1 : 0;
        if (breakTicks_ >= config_.surroundBreakTicks)
        {
            transition(PursuitPhase::Pursuit);
            result.phase = phase_;
            result.event = "surround_broken_reacquire";
            return result;
        }
        bool ready = phaseTicks_ >= config_.surroundMinTicks &&
            ringFeasible(world, config_.captureRadius * 0.72) &&
            surroundGeometrySatisfied(world.target.pose, poses,
                                      config_.surroundRadius,
                                      config_.surroundTolerance,
                                      config_.surroundMinAngleGap,
                                      config_.surroundMaxAngleGap);
        readyTicks_ = ready ? readyTicks_ + 1 : 0;
        if (readyTicks_ >= config_.transitionReadyTicks)
        {
            transition(PursuitPhase::Capture);
            result.phase = phase_;
            result.event = "capture_started";
        }
        return result;
    }

    const bool contained = captureGeometrySatisfied(world.target.pose, poses,
        config_.captureRadius, config_.captureMinAngleGap, config_.captureMaxAngleGap);
    if (contained)
    {
        captureAcquired_ = true;
        ++captureTicks_;
        breakTicks_ = 0;
    }
    else
    {
        captureTicks_ = 0;
        if (captureAcquired_)
            ++breakTicks_;
        else
        {
            bool far = false;
            for (const auto& pursuer : world.pursuers)
                far = far || distanceBetween(world.target.pose, pursuer.pose) > config_.pursuitBreakRadius;
            const bool broken = far || !ringFeasible(world, config_.captureRadius * 0.72);
            breakTicks_ = broken ? breakTicks_ + 1 : 0;
            if (breakTicks_ >= config_.captureAcquisitionBreakTicks)
            {
                transition(far ? PursuitPhase::Pursuit : PursuitPhase::Surround);
                result.phase = phase_;
                result.event = far ? "capture_broken_reacquire" : "capture_broken_reform";
                return result;
            }
        }
    }
    if (captureAcquired_ && breakTicks_ >= config_.captureBreakTicks)
    {
        bool far = false;
        for (const auto& pursuer : world.pursuers)
            far = far || distanceBetween(world.target.pose, pursuer.pose) > config_.pursuitBreakRadius;
        transition(far ? PursuitPhase::Pursuit : PursuitPhase::Surround);
        result.phase = phase_;
        result.event = far ? "capture_broken_reacquire" : "capture_broken_reform";
        return result;
    }
    result.captureProgress = std::min(1.0,
        static_cast<double>(captureTicks_) / std::max(1, config_.captureStableTicks));
    if (captureTicks_ >= config_.captureStableTicks)
    {
        transition(PursuitPhase::Captured);
        result.phase = phase_;
        result.event = "captured";
        result.captureProgress = 1.0;
    }
    return result;
}
