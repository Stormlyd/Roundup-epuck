#ifndef ZOOIDTESTMODE_H
#define ZOOIDTESTMODE_H

#include "ZooidPursuitControl.h"
#include "ZooidPursuitRoles.h"
#include "ZooidPursuitStateMachine.h"

#include <vector>

class ZooidTestMode
{
public:
    bool start(const std::vector<PursuitRobotState>& robots, uint64_t nowMs);
    void stop();
    bool isRunning() const;
    PursuitControlOutput update(const std::vector<PursuitRobotState>& robots,
                                uint64_t nowMs,
                                double fieldWidth,
                                double fieldHeight,
                                uint64_t sequence);
    PursuitRoleMap roleMap() const;
    PursuitStatusSnapshot statusSnapshot() const;

private:
    PursuitControlOutput faultOutput(PursuitFault fault,
                                     const std::vector<PursuitRobotState>& robots);
    PursuitControlOutput stoppedOutput(const std::vector<PursuitRobotState>& robots) const;

    bool running_ = false;
    PursuitRoleMap roles_;
    PursuitStatusSnapshot status_;
    PursuitStateMachine machine_;
    PursuitSlotAssigner slots_;
    WheelCommandSmoother smoother_;
    PursuitPose previousTargetPose_;
    uint64_t previousTargetMs_ = 0;
    bool havePreviousTarget_ = false;
    double estimatedTargetVx_ = 0.0;
    double estimatedTargetVy_ = 0.0;
    std::array<uint64_t, 4> lastAcceptedFeedbackMs_{{0, 0, 0, 0}};
    uint64_t observationSequence_ = 0;
    PursuitControlOutput lastOutput_;
    bool haveLastOutput_ = false;
};

#endif // ZOOIDTESTMODE_H
