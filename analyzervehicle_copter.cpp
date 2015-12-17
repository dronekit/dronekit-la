#include "analyzervehicle_copter.h"

using namespace AnalyzerVehicle;

bool Copter::param_default(const char *name, float &ret) const {
    if (_frame_type == frame_type_quad) {
        auto it = _param_defaults_quad.find(name);
        if (it != _param_defaults_quad.end()) {
            ret = it->second;
            return true;
        }
    }

    auto it = _param_defaults.find(name);
    if (it != _param_defaults.end()) {
        ret = it->second;
        return true;
    }
    return Base::param_default(name, ret);
}

/* I think there's an argument for moving the following into Analyzer: */

bool Copter::is_flying() const
{
    if (! is_armed()) {
        // we hope we're not flying, anyway!
        return false;
    }

    if (! any_motor_running_fast()) {
        return false;
    }

    return true;
}

uint16_t Copter::is_flying_motor_threshold() const
{
    double rc3_min = require_param_with_defaults("RC3_MIN");
    double mot_spin_armed = require_param_with_defaults("MOT_SPIN_ARMED");
    return rc3_min + mot_spin_armed + 10; // the 10 is a random number...
}

bool Copter::any_motor_running_fast() const
{
    for (uint8_t i=1; i<_num_motors; i++) {
        if (_servo_output[i] > is_flying_motor_threshold()) {
            return true;
        }
    }
    return false;
}

std::set<uint8_t> Copter::motors_clipping_high() const
{
    std::set<uint8_t> ret;
    float max;
    if (param("RC3_MAX", max)) {
        for (uint8_t i=1; i<=_num_motors; i++) {
            uint16_t delta = abs((int32_t)_servo_output[i] - (uint16_t)max);
            if ((float)delta/max < .05) { // within 5%
                ret.insert(i);
            }
        }
    }
    return ret;
}

std::set<uint8_t> Copter::motors_clipping_low() const
{
    std::set<uint8_t> ret;
    float min;
    if (param("RC3_MIN", min)) {
        float thr_min;
        if (param("THR_MIN", thr_min)) {
            min += thr_min;
        }
        for (uint8_t i=1; i <= _num_motors; i++) {
            uint16_t delta = abs((int32_t)_servo_output[i] - (uint16_t)min);
            if (_servo_output[i] < (uint16_t)min ||
                ((float)delta/min) < 0.05) {
                ret.insert(i);
            }
            // uint16_t delta = abs((int32_t)_servo_output[i] - (uint16_t)min);
            // ::fprintf(stderr, "%d delta=%d (%f)\n", i, delta, (float)delta/min);
            // if ((float)delta/min < .05) {
            //     ::fprintf(stderr, "%d clipping low \n", i);
            //     ret.insert(i);
            // }
        }
    }
    return ret;
}

void Copter::set_frame_type(copter_frame_type frame_type)
{
    _frame_type = frame_type;
    switch(frame_type) {
    case frame_type_quad:
        _num_motors = 4;
        break;
    case frame_type_y6:
        _num_motors = 6;
        break;
    case invalid:
        ::fprintf(stderr, "Invalid frame type");
        abort();
    }
}

bool Copter::exceeding_angle_max() const
{
    float angle_max; // convert from centidegrees
    if (param_with_defaults("ANGLE_MAX", angle_max)) {
        angle_max /= 100;
        if (fabs(att().roll()) > angle_max) {
            return true;
        }
        if (fabs(att().pitch()) > angle_max) {
            return true;
        }
    }
    return false;
}


void Copter::set_frame(const char *frame_config_string)
{
    if (strstr(frame_config_string, "QUAD")) {
        set_frame_type(AnalyzerVehicle::Copter::frame_type_quad);
    } else if (strstr(frame_config_string, "Y6")) {
        set_frame_type(AnalyzerVehicle::Copter::frame_type_y6);
    } else {
        ::fprintf(stderr, "Unknown frame (%s)\n", frame_config_string);
        abort();
    }
}

