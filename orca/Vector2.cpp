#ifndef HRVO_VECTOR2_H_
#include "Vector2.h"
#endif

#include <ostream>

namespace lzm {
    std::ostream &operator<<(std::ostream &stream, const Vector2 &vector)
    {
        stream << vector.getX() << " " << vector.getY();

        return stream;
    }
}
