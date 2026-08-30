#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_
#pragma once

namespace lzm {

    const float HRVO_EPSILON = 0.00001f;
    const float HRVO_TWO_PI = 6.283185307179586f;

    inline float sqr(float scalar)
    {
        return scalar * scalar;
    }
}

#endif /* DEFINITIONS_H_ */
