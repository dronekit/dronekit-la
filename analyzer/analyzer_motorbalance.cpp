#include "analyzer_motorbalance.h"
#include "analyzervehicle_copter.h"
#include "analyzervehicle_plane.h"

bool Analyzer_MotorBalance::stable()
{
    // check for zero rates (where we then assume thrusts should be equal)
    // we coud also check for copter-roughly-level here?
    float rate_r = _vehicle->rate().roll();
    float rate_p = _vehicle->rate().pitch();
    float rate_y = _vehicle->rate().yaw();

    // ::fprintf(stderr, "rate_r=%f rate_p=%f  rate_y=%f\n", rate_r, rate_p, rate_y);
    if (rate_r > _stable_roll_rate_max ||
        rate_p > _stable_pitch_rate_max ||
        rate_y > _stable_yaw_rate_max) {
        return false;
    }

    return true;
}

bool Analyzer_MotorBalance::all_outputs_same() const
{
    for (uint8_t i=2; i <= _motorcount; i++) {
        const uint8_t servo1_number = _motor_output_map[1];
        const uint8_t servo_number = _motor_output_map[i];
        if (_vehicle->servo_output(servo_number) !=
            _vehicle->servo_output(servo1_number)) {
            return false;
        }
    }
    return true;
}

bool Analyzer_MotorBalance::stabilizing()
{
    if (all_outputs_same()) {
        if (_same_start_time == 0) {
            _same_start_time = _vehicle->T();
        }
        if (_vehicle->T() - _same_start_time > _same_notflying_time) {
            // we are not stabilizing
            // ::fprintf(stderr, "not stabilizing\n");
            _old_stabilizing = false;
            return false;
        }
        return _old_stabilizing;
    }
    _same_start_time = 0;
    // ::fprintf(stderr, "stabilizing\n");
    _old_stabilizing = true;
    return true;
}
void Analyzer_MotorBalance::evaluate_motors()
{
    if (! stable()) {
        _stable_start_T = 0;
        return;
    }

    if (_stable_start_T == 0) {
        _stable_start_T = _vehicle->T();
    }
    if (_vehicle->T() - _stable_start_T > _stable_min_duration) {
        if (!stabilizing()) {
            return;
        }
        for (uint8_t i=1; i <= _motorcount; i++) {
            const uint8_t servo_number = _motor_output_map[i];
            const uint16_t output = _vehicle->servo_output(servo_number);
            _total_pwm[i] += output;
            // ::fprintf(stderr, "servo[%u]=%u ", servo_number, output);
        }
        // ::fprintf(stderr, "\n");
        _sample_count++;
    }
}

void Analyzer_MotorBalance::evaluate_copter()
{
    AnalyzerVehicle::Copter *&copter = (AnalyzerVehicle::Copter*&)_vehicle;

    if (!_done_motor_output_map) {
        switch(copter->frame_type()) {
        case AnalyzerVehicle::Copter::copter_frame_type::frame_type_quad:
            _frame_class = frame_class_quad; // from quadplane.cpp
            _frame_type = frame_type_x; // FIXME!
            _motorcount = motorcount_for_frame_class(_frame_class);
            break;
        default:
            return;
        }

        for(uint8_t i=0; i<16; i++) {
            _motor_output_map[i] = i;
        }
        _done_motor_output_map = true;
    }

    if (!copter->is_flying()) {
        return;
    }

    evaluate_motors();
}

uint8_t Analyzer_MotorBalance::motorcount_for_frame_class(uint8_t value)
{
    uint8_t ret;
    // from quadplane.cpp:
    switch(value) {
    case 0:
        ret = 4;
        break;
    case 1:
        ret = 6;
        break;
    case 2:
        ret = 8;
        break;
    case 3:
        ret = 8;
        break;
    case 4:
        ret = 6;
        break;
    case 7:
        ret = 3;
        break;
    case 10:
        // tailsitter
        ret = 1; // note that this could be incorrect!
        break;
    default:
        ::fprintf(stderr, "Umknown FRAME_CLASS value (%u)\n", value);
        abort();
    }
    return ret;
}

void Analyzer_MotorBalance::evaluate_plane()
{
    AnalyzerVehicle::Plane *&plane = (AnalyzerVehicle::Plane*&)_vehicle;
    if (!_done_motor_output_map) {
        float enable_value;
        if (! plane->param("Q_ENABLE", enable_value) ||
            is_zero(enable_value)) {
            return;
        }
        float frame_class;
        float frame_type;
        if (! plane->param("Q_FRAME_CLASS", frame_class)) {
            return;
        }
        if (! plane->param("Q_FRAME_TYPE", frame_type)) {
            return;
        }
        _frame_class = (frame_class_t)frame_class;
        _frame_type = (frame_type_t)frame_type;

        _motorcount = motorcount_for_frame_class(_frame_class);

        // we assume that RC functions 33 through 36 are quadplane outputs:
        for (uint8_t i=5; i<=16; i++) {
            float value;
            std::string param_name = string_format("RC%u_FUNCTION", i);
            if (! plane->param(param_name, value)) {
                // presumably haven't seen parameter yet
                break;
            }
            if (value >= 33 && value < 33+_motorcount) {
                // i.e. for motor (value-32) look at servo-output i
                const uint8_t motor_num = value-32;
                _motor_output_map[motor_num] = i;
                // ::fprintf(stderr, "Motor %u is on servo-output %u\n", motor_num, _motor_output_map[motor_num]);
            }
        }
        // see if we have an entire map:
        for (uint8_t i=1; i<=_motorcount; i++) {
            if (! _motor_output_map[i]) {
                // ::fprintf(stderr, "no map for (%u)\n", i);
                return;
            }
        }
        _done_motor_output_map = true;
    }

    // check we're in an appropriate mode:
    if (!plane->in_q_mode()) {
        return;
    }

    evaluate_motors();
}

void Analyzer_MotorBalance::evaluate()
{
    switch (_vehicle->vehicletype()) {
    case AnalyzerVehicle::Base::vehicletype_t::copter:
        evaluate_copter();
        break;
    case AnalyzerVehicle::Base::vehicletype_t::plane:
        evaluate_plane();
        break;
    default:
        break;
    }
}

void Analyzer_MotorBalance::add_axis_result(const char *axis, const uint16_t imbalance, const uint16_t imbalance_warn, const uint16_t imbalance_fail)
{
    Analyzer_MotorBalance_Result_Axis *result = new Analyzer_MotorBalance_Result_Axis();
    result->add_source(_data_sources.get("RATE"));
    result->add_source(_data_sources.get("SERVO_OUTPUT"));
    result->set_reason(string_format("Imbalance detected in %s axis", axis, imbalance));
    result->set_axis(axis);
    result->set_delta(imbalance);
    if (imbalance >= imbalance_fail) {
        result->set_status(analyzer_status_fail);
        result->set_severity_score(50);
    } else if (imbalance >= imbalance_warn) {
        result->set_status(analyzer_status_warn);
        result->set_severity_score(25);
    }
    add_result(result);
}

void Analyzer_MotorBalance::end_of_log_motors_x(const uint16_t *const averages)
{
//    AnalyzerVehicle::Copter *&copter = (AnalyzerVehicle::Copter*&)_vehicle;
    const int16_t motor1 = averages[1];
    const int16_t motor2 = averages[2];
    const int16_t motor3 = averages[3];
    const int16_t motor4 = averages[4];
    // ::fprintf(stderr, "end of log quad\n");
    const uint16_t delta_roll = abs((motor1 + motor4) - (motor2 + motor3))/2;
    const uint16_t delta_pitch = abs((motor1 + motor3) - (motor2 + motor4))/2;
    const uint16_t delta_yaw = abs((motor1 + motor2) - (motor3 + motor4))/2;
    // ::fprintf(stderr, "delta_roll=%u delta_pitch=%u delta_yaw=%u\n", delta_roll, delta_pitch, delta_yaw);
    if (delta_roll > _attitude_pwm_delta_warn) {
        add_axis_result("Roll", delta_roll, _attitude_pwm_delta_warn, _attitude_pwm_delta_fail);
    }
    if (delta_pitch > _attitude_pwm_delta_warn) {
        add_axis_result("Pitch", delta_pitch, _attitude_pwm_delta_warn, _attitude_pwm_delta_fail);
    }
    if (delta_yaw > _yaw_pwm_delta_warn) {
        add_axis_result("Yaw", delta_yaw, _yaw_pwm_delta_warn, _yaw_pwm_delta_fail);
    }
}


void Analyzer_MotorBalance::add_motor_result(const uint8_t motornum, uint16_t motorsaverage, uint16_t motoraverage)
{
    Analyzer_MotorBalance_Result_Motor *result = new Analyzer_MotorBalance_Result_Motor();
    result->add_source(_data_sources.get("RATE"));
    result->add_source(_data_sources.get("SERVO_OUTPUT"));
    result->set_reason(string_format("Imbalance detected on motor %u", motornum));
    result->add_evidence(string_format("average-this-motor=%u", motoraverage));
    result->add_evidence(string_format("average-all-motors=%u", motorsaverage));
    int32_t delta = motoraverage - motorsaverage;
    if (abs(delta) >= _motor_imbalance_fail) {
        result->set_status(analyzer_status_fail);
        result->set_severity_score(40);
    } else if (abs(delta) > _motor_imbalance_warn) {
        result->set_status(analyzer_status_warn);
        result->set_severity_score(20);
    }
    add_result(result);
}

void Analyzer_MotorBalance::end_of_log_motors()
{
    if (_sample_count == 0) {
        return;
    }
    // ::fprintf(stderr, "samples=%i\n", _sample_count);
    uint16_t averages[16];
    uint64_t total = 0;
    for (uint8_t i=1; i<=_motorcount;i++) {
        averages[i] = _total_pwm[i]/_sample_count;
        total += _total_pwm[i];
        // ::fprintf(stderr, "average[%u]=%i\n", i, averages[i]);
    }

    uint16_t average = total / _sample_count / _motorcount;
    // ::fprintf(stderr, "average=%u\n", average);
    for (uint8_t i=1; i<=_motorcount;i++) {
        int16_t delta = average - averages[i];
        if (abs(delta) > _motor_imbalance_warn) {
            add_motor_result(i, average, averages[i]);
        }
    }

    switch (_frame_type) {
    case frame_type_x:
        end_of_log_motors_x(averages);
        break;
    default:
        break;
    }
}
void Analyzer_MotorBalance::end_of_log_copter()
{
    end_of_log_motors();
}
void Analyzer_MotorBalance::end_of_log_plane()
{
    end_of_log_motors();
}

void Analyzer_MotorBalance::end_of_log(const uint32_t packet_count UNUSED)
{
    switch (_vehicle->vehicletype()) {
    case AnalyzerVehicle::Base::vehicletype_t::copter:
        end_of_log_copter();
        break;
    case AnalyzerVehicle::Base::vehicletype_t::plane:
        end_of_log_plane();
        break;
    default:
        break;
    }
}
