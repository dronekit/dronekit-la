#include "analyzervehicle_copter.h"

using namespace AnalyzerVehicle;

bool Copter::is_flying() {
    if (! is_armed()) {
        // we hope we're not flying, anyway!
        return false;
    }

    if (! any_motor_running_fast()) {
        return false;
    }

    return true;
}

bool Copter::any_motor_running_fast() {
    for (uint8_t i=1; i<_num_motors; i++) {
        if (_servo_output[i] > is_flying_motor_threshold) {
            return true;
        }
    }
    return false;
}
void Copter::handle_decoded_message(uint64_t T, mavlink_statustext_t &msg) {
    Base::handle_decoded_message(T, msg);

    if (strstr(msg.text, "Frame")) {
        if (strstr(msg.text, "QUAD")) {
            _num_motors = 4;
        }
    }
}

bool Copter::exceeding_angle_max()
{
    if (!seen_parameter("ANGLE_MAX")) {
        return false;
    }

    float angle_max = params["ANGLE_MAX"] / 100; // convert from centidegrees
    float angle_max_radians = angle_max/180.0 * M_PI;

    ::printf("att.roll=%f\n", att().roll());
    if (att().roll() > angle_max_radians) {
        return true;
    }
    if (att().pitch() > angle_max_radians) {
        return true;
    }
    return false;
}


void Copter::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg)
{
    Base::handle_decoded_message(T, msg);

    _servo_output[1] = msg.servo1_raw;
    _servo_output[2] = msg.servo2_raw;
    _servo_output[3] = msg.servo3_raw;
    _servo_output[4] = msg.servo4_raw;
    _servo_output[5] = msg.servo5_raw;
    _servo_output[6] = msg.servo6_raw;
    _servo_output[7] = msg.servo7_raw;
    _servo_output[8] = msg.servo8_raw;
}
