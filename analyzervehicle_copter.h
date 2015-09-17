#include "analyzervehicle.h"

#ifndef _ANALYZER_VEHICLE_COPTER
#define _ANALYZER_VEHICLE_COPTER

#include <set>

namespace AnalyzerVehicle {

    class Copter : public Base {
    public:
        Copter() :
            Base()
            { }
        bool is_flying();
        
        uint8_t num_motors() { return _num_motors; }

        bool any_motor_running_fast();
        bool exceeding_angle_max();

        std::set<uint8_t> motors_clipping_low();
        std::set<uint8_t> motors_clipping_high();
        
        bool param_default(const char *name, float &ret);

        static const uint16_t is_flying_motor_threshold = 1250;

        vehicletype_t vehicletype() override {
            return copter;
        }

        enum copter_frame_type {
            invalid = 0,
            frame_type_quad = 19,
            frame_type_y6,
        };
        void set_frame_type(copter_frame_type frame_type);

        void set_frame(const char *frame_config_string);

        copter_frame_type frame_type() { return _frame_type; }
    protected:

    private:
        copter_frame_type _frame_type = invalid;
        std::map<const std::string, float> _param_defaults = {
            { "ANGLE_MAX", 3000.0f } // degrees*100
        };
        std::map<const std::string, float> _param_defaults_quad = {
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

}; // end namepsace

#endif
