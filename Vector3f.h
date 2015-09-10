#ifndef _VECTOR3F_H
#define _VECTOR3F_H

class Vector3f {
public:
    float& operator [](size_t offset) {
        return vals[offset];
    };

    float vals[3];
};

#endif
