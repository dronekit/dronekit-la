#include "analyzervehicle.h"

#ifndef _ANALYZER_VEHICLE_PLANE
#define _ANALYZER_VEHICLE_PLANE

#include <set>

namespace AnalyzerVehicle {

    class Plane : public Base {
    public:
        Plane() :
            Base()
            { }

        const std::string typeString() const { return "Plane"; }

        bool is_flying();

        bool param_default(const char *name, float &ret);

        static const uint16_t is_flying_motor_threshold = 1250;

        vehicletype_t vehicletype() override {
            return plane;
        }

    protected:

    private:
        std::map<const std::string, float> _param_defaults = {
            // { "ANGLE_MAX", 3000.0f } // degrees*100
        };
    };

} // end namepsace

#endif
