#include "analyzervehicle.h"

#ifndef _ANALYZER_VEHICLE_PLANE
#define _ANALYZER_VEHICLE_PLANE

#include <set>

namespace AnalyzerVehicle {

    class Plane : public Base {
    public:

        /// @brief Construct a Plane object
        Plane() :
            Base()
            { }

        /// @brief Returns a short string describing the vehicle.
        /// @return A short string describing the vehicle e.g. "Copter".
        const std::string typeString() const override { return "Plane"; }

        /// @brief Evaluation of whether the vehicle is flying.
        /// @return ``true`` if it is believed the vehicle is flying.
        bool is_flying() const override;


        /// @brief Return the default value for a parameter.
        /// @detail Default parameters are supplied so that some analysis can be done even in the absence of parameters in the logs.
        /// @param name Parameter to retrieve default value for.
        /// @param[out] ret Value of parameter.
        /// @return ``true`` if a default was found for the parameters.
        bool param_default(const char *name, float &ret) const override;

        /// @brief Vehicle type.
        /// @return The current vehicle type.
        vehicletype_t vehicletype() const override {
            return plane;
        }

    protected:

    private:

        // parameter defaults:
        std::map<const std::string, float> _param_defaults = {
            // { "ANGLE_MAX", 3000.0f } // degrees*100
        };
    };

} // end namepsace

#endif
