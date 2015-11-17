#include "analyzervehicle_plane.h"

using namespace AnalyzerVehicle;

bool Plane::param_default(const char *name, float &ret) const
{
    auto it = _param_defaults.find(name);
    if (it != _param_defaults.end()) {
        ret = it->second;
        return true;
    }

    return Base::param_default(name, ret);
}

/* I think there's an argument for moving the following into Analyzer: */

bool Plane::is_flying() const
{
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
