#ifndef ZOOIDPURSUITSTATEMACHINE_H
#define ZOOIDPURSUITSTATEMACHINE_H

#include "ZooidPursuitTypes.h"

struct PursuitConfig
{
    double pursuitRadius = 0.36;
    double pursuitHoldRadius = 0.31;
    int pursuitMinTicks = 35;
    double surroundRadius = 0.24;
    double surroundTolerance = 0.055;
    int surroundMinTicks = 35;
    double surroundMinAngleGap = 0.9;
    double surroundMaxAngleGap = 3.091592653589793;
    double captureRadius = 0.17;
    double captureMinAngleGap = 1.65;
    double captureMaxAngleGap = 2.60;
    int captureStableTicks = 20;
    int transitionReadyTicks = 3;
    double pursuitBreakRadius = 0.46;
    int surroundBreakTicks = 8;
    int captureBreakTicks = 6;
    int captureAcquisitionBreakTicks = 8;
};

class PursuitStateMachine
{
public:
    explicit PursuitStateMachine(const PursuitConfig& config = PursuitConfig());
    bool start();
    void stop();
    void reset();
    PursuitPhase phase() const;
    PursuitPhaseResult update(const PursuitWorldState& world);

private:
    bool acceptFresh(const PursuitWorldState& world);
    void transition(PursuitPhase phase);
    bool ringFeasible(const PursuitWorldState& world, double radius) const;

    PursuitConfig config_;
    PursuitPhase phase_ = PursuitPhase::Idle;
    int phaseTicks_ = 0;
    int readyTicks_ = 0;
    int captureTicks_ = 0;
    int breakTicks_ = 0;
    uint64_t lastSequence_ = 0;
    uint64_t lastStampMs_ = 0;
    bool haveObservation_ = false;
    bool captureAcquired_ = false;
};

#endif // ZOOIDPURSUITSTATEMACHINE_H
