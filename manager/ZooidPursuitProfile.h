#ifndef ZOOIDPURSUITPROFILE_H
#define ZOOIDPURSUITPROFILE_H

#include <cstdint>

// Single source of truth for the physical pursuit demo. Values are SI units
// unless the name says otherwise, so later hardware tuning does not desync the
// controller from the state-machine acceptance geometry.
namespace PursuitProfile
{
constexpr double BoundaryMargin = 0.057;
constexpr double BoundaryBrakingBand = 0.030;

constexpr double PursuitTransitionRadius = 0.36;
constexpr double PursuitHoldRadius = 0.31;
constexpr double SurroundRadius = 0.24;
constexpr double CaptureGoalRadius = 0.17 * 0.72;
constexpr double CaptureRadius = 0.17;
constexpr double PursuitBreakRadius = 0.46;

constexpr double PursuerPursuitSpeed = 0.100;
constexpr double PursuerSurroundSpeed = 0.085;
constexpr double PursuerCaptureSpeed = 0.070;
constexpr double TargetPursuitSpeed = 0.040;
constexpr double TargetPressureSpeed = 0.050;
constexpr double TargetSurroundSpeed = 0.035;
constexpr double TargetCaptureSpeed = 0.025;
constexpr double CapturePressureRadius = 0.201875;

constexpr double CollisionInfluenceRadius = 0.16;
constexpr double CollisionAttractionLength = 0.12;
constexpr double CollisionRepulsionGain = 6.0;
constexpr double CollisionTangentialGain = 1.5;
constexpr double TargetTransitClearance = 0.14;
constexpr double CaptureTargetTransitClearance = 0.105;
constexpr double TargetTransitBearingStep = 0.40;
constexpr double TargetMotionGuardRadius = 0.13;
// Optical position feedback is quantized, so a steady 0.04 m/s target can
// arrive as alternating zero and roughly 0.10 m/s samples. Keep the outlier
// cap above that quantization pulse before applying the low-pass filter.
constexpr double TargetVelocityEstimateLimit = 0.120;
constexpr std::uint64_t CommandHoldTimeoutMs = 100;
constexpr std::uint64_t FeedbackTimeoutMs = 250;
}

#endif // ZOOIDPURSUITPROFILE_H
