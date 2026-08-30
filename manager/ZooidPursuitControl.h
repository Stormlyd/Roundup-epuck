#ifndef ZOOIDPURSUITCONTROL_H
#define ZOOIDPURSUITCONTROL_H

#include "ZooidPursuitTypes.h"
#include "ZooidPursuitGeometry.h"

#include <map>

double receiverHeadingToYaw(double receiverDegrees);
PursuitTwist driveToward(const PursuitPose& current,
                         const PursuitPose& goal,
                         double maxLinear,
                         double maxAngular = 1.8);
WheelCommand differentialDrive(const PursuitTwist& twist,
                               double wheelBase = 0.05,
                               double unitsPerMetrePerSecond = 1000.0);

class WheelCommandSmoother
{
public:
    explicit WheelCommandSmoother(int maxDelta = 40);
    WheelCommand smooth(unsigned int id, WheelCommand desired);
    void clear();

private:
    int maxDelta_;
    std::map<unsigned int, WheelCommand> previous_;
};

PursuitTwist targetEscapeCommand(const PursuitWorldState& world,
                                 PursuitPhase phase);
PursuitControlOutput computePursuitCommands(
    const PursuitWorldState& world,
    const PursuitRoleMap& roles,
    PursuitPhase phase,
    PursuitSlotAssigner& slots,
    WheelCommandSmoother& smoother);

#endif // ZOOIDPURSUITCONTROL_H
