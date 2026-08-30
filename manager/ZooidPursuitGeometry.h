#ifndef ZOOIDPURSUITGEOMETRY_H
#define ZOOIDPURSUITGEOMETRY_H

#include "ZooidPursuitTypes.h"

#include <array>

double normalizeAngle(double angle);
double distanceBetween(const PursuitPose& first, const PursuitPose& second);
std::array<PursuitPose, 3> makeTriangularRing(const PursuitPose& target,
                                              double radius,
                                              double headingOffset = 0.0);
std::array<double, 3> circularAngularGaps(
    const PursuitPose& target,
    const std::array<PursuitPose, 3>& poses);
bool ringInsideBounds(const std::array<PursuitPose, 3>& ring,
                      double width,
                      double height,
                      double margin);
bool captureGeometrySatisfied(const PursuitPose& target,
                              const std::array<PursuitPose, 3>& pursuers,
                              double captureRadius,
                              double minAngleGap,
                              double maxAngleGap);
bool surroundGeometrySatisfied(const PursuitPose& target,
                               const std::array<PursuitPose, 3>& pursuers,
                               double radius,
                               double tolerance,
                               double minAngleGap,
                               double maxAngleGap);

class PursuitSlotAssigner
{
public:
    bool assign(const PursuitWorldState& world,
                PursuitPhase phase,
                double radius,
                bool nonExpanding,
                std::array<PursuitPose, 3>& goals);
    void clear();

private:
    PursuitPhase phase_ = PursuitPhase::Idle;
    std::array<double, 3> bearings_{{0.0, 0.0, 0.0}};
    bool remembered_ = false;
};

#endif // ZOOIDPURSUITGEOMETRY_H
