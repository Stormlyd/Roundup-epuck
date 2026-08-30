#ifndef ZOOIDSPEEDCODEC_H
#define ZOOIDSPEEDCODEC_H

#include <cstdint>

struct EncodedWheelSpeeds
{
    int16_t left;
    int16_t right;
    uint16_t positionX;
    uint16_t positionY;
};

EncodedWheelSpeeds encodeWheelSpeeds(int left, int right);

#endif // ZOOIDSPEEDCODEC_H
