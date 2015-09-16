#ifndef _VECTOR3F_H
#define _VECTOR3F_H

#include <math.h>

class Vector3f {
public:
    float& operator [](size_t offset) {
        return vals[offset];
    };

    double len() const {
        return sqrt(vals[0]*vals[0] + vals[1]*vals[1] + vals[2]*vals[2]);
    }

private:
    float vals[3];
};

#endif
