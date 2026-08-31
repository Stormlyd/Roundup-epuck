#ifndef ZOOIDCBFSAFETY_H
#define ZOOIDCBFSAFETY_H

#include <cstdint>
#include <map>
#include <vector>

#include "ZooidPursuitTypes.h"

struct CbfRobotState
{
    unsigned int id = 0;
    PursuitPose pose;
    uint64_t feedbackMs = 0;
};

struct ZooidCbfConfig
{
    double fieldWidth = ZooidFieldWidth;
    double fieldHeight = ZooidFieldHeight;
    double safeRadius = 0.050;
    double minimumDistance = 0.100;
    double gamma = 4.0;
    double commandUnitsPerMetrePerSecond = 1000.0;
    uint64_t freshnessLimitMs = 100;
    uint64_t maximumSnapshotSkewMs = 50;
    int maximumWheelCommand = 1000;
    unsigned int maximumIterations = 256;
    double tolerance = 1e-9;
};

enum class ZooidCbfStatus
{
    Safe,
    Intervened,
    InvalidInput,
    UnsafeState,
    SolverFailure
};

struct ZooidCbfResult
{
    ZooidCbfStatus status = ZooidCbfStatus::InvalidInput;
    std::map<unsigned int, WheelCommand> commands;
};

ZooidCbfResult applyZooidCbf(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& nominal,
    uint64_t nowMs,
    const ZooidCbfConfig& config = {});

bool zooidCbfConstraintsSatisfied(
    const std::vector<CbfRobotState>& robots,
    const std::map<unsigned int, WheelCommand>& commands,
    const ZooidCbfConfig& config = {});

#endif // ZOOIDCBFSAFETY_H
