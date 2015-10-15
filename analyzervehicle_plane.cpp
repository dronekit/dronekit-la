#include "analyzervehicle_plane.h"

using namespace AnalyzerVehicle;

bool Plane::param_default(const char *name, float &ret) {
    if (_param_defaults[name]) {
        ret = _param_defaults[name];
        return true;
    }
    return Base::param_default(name, ret);
}

/* I think there's an argument for moving the following into Analyzer: */

bool Plane::is_flying() {
    if (! is_armed()) {
        // we hope we're not flying, anyway!
        return false;
    }

    // worst is_flying ever:
    double relalt;
    if (relative_alt(relalt)) {
        if (relalt > 15) {
            return true;
        }
    }

    return false;
}
