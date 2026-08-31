#include "ZooidCoordinates.h"

#include <cmath>

bool zooidRawToWorld(uint16_t rawX, uint16_t rawY, ZooidWorldPoint& world)
{
    if (rawX < ZooidRawMinX || rawX > ZooidRawMaxX ||
        rawY < ZooidRawMinY || rawY > ZooidRawMaxY) {
        return false;
    }

    world.x = ZooidFieldWidth *
        (static_cast<double>(ZooidRawMaxX - rawX) /
         static_cast<double>(ZooidRawMaxX - ZooidRawMinX));
    world.y = ZooidFieldHeight *
        (static_cast<double>(rawY - ZooidRawMinY) /
         static_cast<double>(ZooidRawMaxY - ZooidRawMinY));
    return true;
}

bool zooidWorldToRaw(const ZooidWorldPoint& world,
                     uint16_t& rawX,
                     uint16_t& rawY)
{
    if (!std::isfinite(world.x) || !std::isfinite(world.y) ||
        world.x < 0.0 || world.x > ZooidFieldWidth ||
        world.y < 0.0 || world.y > ZooidFieldHeight) {
        return false;
    }

    rawX = static_cast<uint16_t>(std::lround(
        static_cast<double>(ZooidRawMaxX) -
        world.x * static_cast<double>(ZooidRawMaxX - ZooidRawMinX) /
            ZooidFieldWidth));
    rawY = static_cast<uint16_t>(std::lround(
        static_cast<double>(ZooidRawMinY) +
        world.y * static_cast<double>(ZooidRawMaxY - ZooidRawMinY) /
            ZooidFieldHeight));
    return true;
}
