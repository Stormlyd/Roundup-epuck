#ifndef ZOOIDPURSUITCONTROL_H
#define ZOOIDPURSUITCONTROL_H

#include "ZooidPursuitTypes.h"
#include "ZooidPursuitGeometry.h"

#include <map>

namespace PursuitDriveCalibration
{
// e-puck2 drivetrain: 41 mm wheels, 53 mm track, 1000 motor steps/revolution.
// Wheel commands are steps/second, so m/s must be multiplied by steps/metre.
constexpr double Pi = 3.14159265358979323846;
constexpr double WheelDiameterMetres = 0.041;
constexpr double WheelBaseMetres = 0.053;
constexpr double WheelUnitsPerRevolution = 1000.0;
constexpr double WheelUnitsPerMetre =
    WheelUnitsPerRevolution / (Pi * WheelDiameterMetres);
}

double receiverHeadingToYaw(double receiverDegrees);
PursuitTwist driveToward(const PursuitPose& current,
                         const PursuitPose& goal,
                         double maxLinear,
                         double maxAngular = 1.8);
PursuitTwist brakeAtFieldBoundary(const PursuitPose& current,
                                  double fieldWidth,
                                  double fieldHeight,
                                  PursuitTwist twist);
WheelCommand differentialDrive(const PursuitTwist& twist,
                               double wheelBase = PursuitDriveCalibration::WheelBaseMetres,
                               double unitsPerMetrePerSecond =
                                   PursuitDriveCalibration::WheelUnitsPerMetre);

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
