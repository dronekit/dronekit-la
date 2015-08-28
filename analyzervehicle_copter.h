#include "analyzervehicle.h"

#ifndef _ANALYZER_VEHICLE_COPTER
#define _ANALYZER_VEHICLE_COPTER

namespace AnalyzerVehicle {

    class Copter : public Base {
    public:
        Copter() :
            _num_motors(0),
            _servo_output{ 0 }
            { }
        bool is_flying();
        void handle_decoded_message(uint64_t T, mavlink_statustext_t &msg);
        void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg);
        
        bool any_motor_running_fast();
        bool exceeding_angle_max();

        uint8_t _num_motors; // e.g. 4 for a quad...
        uint16_t _servo_output[9]; // indexed from 1
        static const uint16_t is_flying_motor_threshold = 1250;
    protected:

    private:
    };

}; // end namepsace

#endif
