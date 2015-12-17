#include "analyzervehicle.h"

#ifndef _ANALYZER_VEHICLE_COPTER
#define _ANALYZER_VEHICLE_COPTER

#include <set>

namespace AnalyzerVehicle {

    class Copter : public Base {
    public:

        /// @brief Construct a Copter object
        Copter() :
            Base()
            { }

        /// @brief Returns a short string describing the vehicle.
        /// @return A short string describing the vehicle e.g. "Copter".
        const std::string typeString() const override { return "Copter"; }

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
            return copter;
        }

        /// @brief Supply number of motors configured on this vehicle.
        uint8_t num_motors() const { return _num_motors; }

        /// @brief Indicate if any of the vehicle motors appears to be running at flight speed.
        /// @return True if any motor is running fast enough to indicate flight.
        bool any_motor_running_fast() const;

        /// @brief Indicate excessive lean angles of aircraft.
        /// @return True if vehicle is past its configured maximum angle.
        bool exceeding_angle_max() const;

        /// @brief Indicates if any motor appears to be running as slow as it can
        /// @return a set of motor numbers which appear to be running slowly
        std::set<uint8_t> motors_clipping_low() const;
        /// @brief Indicates if any motor appears to be running as fast as it can
        /// @return a set of motor numbers which appear to be running flat out
        std::set<uint8_t> motors_clipping_high() const;

        /// @brief Supply threshold over which a motor is considered to be running fast enough that it might cause flight
        /// @return a PWM output value
        uint16_t is_flying_motor_threshold() const;

        /// @brief all possible copter frame types
        enum copter_frame_type {
            invalid = 0,
            frame_type_quad = 19,
            frame_type_y6,
        };
        /// @brief set copter frame type
        /// @param frame_type new copter frame type
        void set_frame_type(copter_frame_type frame_type);

        /// @brief set copter frame type
        /// @param frame_config_string new copter frame type, as a string as found in dataflash and mavlink logs
        void set_frame(const char *frame_config_string);

        /// @brief retrieve copter frame type
        /// @return the current copter frame type
        copter_frame_type frame_type() { return _frame_type; }

    protected:

    private:
        copter_frame_type _frame_type = invalid;

        // parameter defaults:
        std::map<const std::string, float> _param_defaults = {
            { "ANGLE_MAX", 3000.0f }, // degrees*100
            { "MOT_SPIN_ARMED", 75.0f } // pwm-delta
        };
        std::map<const std::string, float> _param_defaults_quad = {
            { "RC3_MIN", 1000.0f },
            { "RCOU1_MIN", 1200.0f },
            { "RCOU1_MAX", 1800.0f },
            { "RCOU2_MIN", 1200.0f },
            { "RCOU2_MAX", 1800.0f },
            { "RCOU3_MIN", 1200.0f },
            { "RCOU3_MAX", 1800.0f },
            { "RCOU4_MIN", 1200.0f },
            { "RCOU4_MAX", 1800.0f }
        };

        uint8_t _num_motors = 0; // e.g. 4 for a quad...
    };

} // end namepsace

#endif
