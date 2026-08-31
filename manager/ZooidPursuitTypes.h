#ifndef ZOOIDPURSUITTYPES_H
#define ZOOIDPURSUITTYPES_H

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "ZooidCoordinates.h"
#include "ZooidWheelCommand.h"

enum class PursuitPhase
{
    Idle,
    Pursuit,
    Surround,
    Capture,
    Captured
};

enum class PursuitFault
{
    None,
    NotEnoughRobots,
    ParticipantMissing,
    FeedbackStale,
    InvalidFeedback,
    InvalidGeometry,
    SafetyViolation,
    ReceiverError,
    ManualStop
};

struct PursuitPose
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct PursuitTwist
{
    double linear = 0.0;
    double angular = 0.0;
};

struct PursuitRobotState
{
    unsigned int id = 0;
    PursuitPose pose;
    uint64_t feedbackMs = 0;
    bool connected = false;
    bool activated = false;
    double vx = 0.0;
    double vy = 0.0;
};

struct PursuitRoleMap
{
    unsigned int targetId = 0;
    std::array<unsigned int, 3> pursuerIds{{0, 0, 0}};
    bool valid = false;

    std::vector<unsigned int> participantIds() const;
};

struct PursuitWorldState
{
    PursuitRobotState target;
    std::array<PursuitRobotState, 3> pursuers;
    double fieldWidth = ZooidFieldWidth;
    double fieldHeight = ZooidFieldHeight;
    uint64_t stampMs = 0;
    uint64_t sequence = 0;
};

struct PursuitPhaseResult
{
    PursuitPhase phase = PursuitPhase::Idle;
    std::string event;
    double captureProgress = 0.0;
};

struct PursuitControlOutput
{
    std::map<unsigned int, WheelCommand> commands;
    PursuitPhase phase = PursuitPhase::Idle;
    PursuitFault fault = PursuitFault::None;
    std::string event;
    double captureProgress = 0.0;
};

struct PursuitStatusSnapshot
{
    PursuitPhase phase = PursuitPhase::Idle;
    PursuitFault fault = PursuitFault::None;
    PursuitRoleMap roles;
    std::array<double, 3> targetDistances{{0.0, 0.0, 0.0}};
    double captureProgress = 0.0;
    std::string event;
};

#endif // ZOOIDPURSUITTYPES_H
