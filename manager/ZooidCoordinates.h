#ifndef ZOOIDCOORDINATES_H
#define ZOOIDCOORDINATES_H

#include <cstdint>

constexpr double ZooidFieldWidth = 1.460;
constexpr double ZooidFieldHeight = 0.914;
constexpr uint16_t ZooidRawMinX = 63;
constexpr uint16_t ZooidRawMaxX = 960;
constexpr uint16_t ZooidRawMinY = 229;
constexpr uint16_t ZooidRawMaxY = 795;

struct ZooidWorldPoint
{
    double x;
    double y;
};

bool zooidRawToWorld(uint16_t rawX, uint16_t rawY, ZooidWorldPoint& world);
bool zooidWorldToRaw(const ZooidWorldPoint& world,
                     uint16_t& rawX,
                     uint16_t& rawY);

#endif // ZOOIDCOORDINATES_H
