#ifndef _VECTOR3F_H
#define _VECTOR3F_H

#include <math.h>

class Vector3f {
public:
    float& operator [](size_t offset) {
        return vals[offset];
    };

    float operator [](size_t offset) const {
        return vals[offset];
    };

    Vector3f operator +=(const Vector3f other) {
        vals[0] += other[0];
        vals[1] += other[1];
        vals[2] += other[2];
        return *this;
    };

    Vector3f operator /=(const uint64_t divisor) {
        vals[0] /= (float)divisor;
        vals[1] /= (float)divisor;
        vals[2] /= (float)divisor;
        return *this;
    };

    Vector3f operator -(const Vector3f other) {
        Vector3f ret;
        ret[0] = vals[0] - other[0];
        ret[1] = vals[1] - other[1];
        ret[2] = vals[2] - other[2];
        return ret;
    };

    double len() const {
        return sqrt(vals[0]*vals[0] + vals[1]*vals[1] + vals[2]*vals[2]);
    }

private:
    float vals[3];
};

#endif
