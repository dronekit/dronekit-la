#include "analyzervehicle_rover.h"

using namespace AnalyzerVehicle;

bool Rover::param_default(const char *name, float &ret) const
{
    auto it = _param_defaults.find(name);
    if (it != _param_defaults.end()) {
        ret = it->second;
        return true;
    }

    return Base::param_default(name, ret);
}

/* I think there's an argument for moving the following into Analyzer: */

bool Rover::is_flying() const
{
    return false;
}
