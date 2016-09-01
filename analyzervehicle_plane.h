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

        typedef enum {
            UNKNOWN=65536,
            MANUAL = 0,
            QSTABILIZE = 17,
            QHOVER = 18,
            QLOITER = 19,
            QLAND = 20,
            QRTL = 21,
        } flightmode_t;

        /// @brief Return current flight mode
        /// @return current flight mode
        flightmode_t mode() const { return _mode; }
        /// @brief set current flight mode
        void set_mode(const flightmode_t mode) { _mode = mode; }

        /// @brief Return true currently in vertical flight mode
        /// @return true if in vertical flight mode, false otherwise
        bool in_q_mode() const {
            return (mode() == QSTABILIZE ||
                    mode() == QHOVER ||
                    mode() == QLOITER ||
                    mode() == QLAND ||
                    mode() == QRTL);
        }

    protected:

    private:

        // parameter defaults:
        std::map<const std::string, float> _param_defaults = {
            // { "ANGLE_MAX", 3000.0f } // degrees*100
        };

        flightmode_t _mode = UNKNOWN;
    };

} // end namepsace

#endif
