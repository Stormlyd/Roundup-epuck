#include "ZooidSpeedCodec.h"

namespace
{
int16_t clampWheelSpeed(int value)
{
    if (value > 1000) return 1000;
    if (value < -1000) return -1000;
    return static_cast<int16_t>(value);
}
}

EncodedWheelSpeeds encodeWheelSpeeds(int left, int right)
{
    const int16_t safeLeft = clampWheelSpeed(left);
    const int16_t safeRight = clampWheelSpeed(right);
    return {
        safeLeft,
        safeRight,
        static_cast<uint16_t>(safeLeft + 2007),
        static_cast<uint16_t>(safeRight + 2007)
    };
}
