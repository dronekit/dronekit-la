#include "analyzer_notcrashed.h"

#include "util.h"

#include "analyzer_util.h"
#include "analyzervehicle_copter.h"

void Analyzer_NotCrashed::end_of_log(const uint32_t packet_count UNUSED)
{
    if (_result != NULL) {
        add_result(_result);
        _result = NULL;
    }
    if (_vehicle->roll_modtime() == 0) {
        _result = new Analyzer_NotCrashed_Result();
        _result->add_source(_data_sources.get("ATTITUDE"));
        _result->set_reason("Vehicle's attitude never updated");
        _result->set_status(analyzer_status_warn);
        add_result(_result);
        _result = NULL;
    } else if (!result_count()) {
        _result = new Analyzer_NotCrashed_Result();
        _result->set_reason("Never crashed");
        _result->add_source(_data_sources.get("ATTITUDE"));
        _result->add_source(_data_sources.get("SERVO_OUTPUT"));
        _result->add_source(_data_sources.get("PARAM"));
        _result->add_source(_data_sources.get("CRASH"));
        _result->set_status(analyzer_status_ok);
        add_result(_result);
        _result = NULL;
    }
}

void Analyzer_NotCrashed::evaluate_attitude()
{
    if (_vehicle->vehicletype() != AnalyzerVehicle::Base::copter) {
        // need to subclass this class to do notcrashed properly
        return;
    }
    AnalyzerVehicle::Copter *v = (AnalyzerVehicle::Copter*&)_vehicle;

    // {
    //     float angle_max = 0;
    //     v->param_with_defaults("ANGLE_MAX", angle_max);
    //     ::fprintf(stderr, "angle_max = %f angle= %f\n",angle_max, (fabs(v->roll()) > fabs(v->pitch())) ? v->roll() : v->pitch());
    // }
    bool crashed = v->exceeding_angle_max() &&
        v->any_motor_running_fast();
    if (_result == NULL) {
        if (crashed) {
            _result = new Analyzer_NotCrashed_Result();
            _result->set_status(analyzer_status_fail);
            _result->set_reason("Vehicle is past maximum allowed angle and running its motors");
            _result->set_T(_vehicle->T());
            _result->angle = (fabs(v->roll()) > fabs(v->pitch())) ? v->roll() : v->pitch();
            if (v->param_with_defaults("ANGLE_MAX", _result->angle_max)) {
                _result->angle_max /= 100; // 100* degrees to degrees
                for (uint8_t i=1; i<=v->num_motors(); i++) {
                    _result->servo_output[i] = v->servo_output(i);
                }
            }
            _result->add_evidence(string_format("ANGLE_MAX (fabs(%f) > %f)", _result->angle, _result->angle_max));
            for (uint8_t i=1; i<=v->num_motors(); i++) {
                if (v->servo_output(i) > v->is_flying_motor_threshold()) {
                    _result->add_evidence(string_format("SERVO_OUTPUT_RAW.servo%d_raw=%f",
                                                        i, _result->servo_output[i]));
                }
            }

            _result->add_source(_data_sources.get("ATTITUDE"));
            _result->add_source(_data_sources.get("SERVO_OUTPUT"));
            _result->add_source(_data_sources.get("PARAM"));
            _result->increase_severity_score(100);
        }
    } else { // _result is set, incident underway
        if (!crashed) {
            // incident done
            if (_vehicle->T() - _result->T() > 1000000) {
                // accept this result
                add_result(_result);
            }
            _result = NULL;
        }
    }
}

void Analyzer_NotCrashed::evaluate_vehicle()
{
    if (_vehicle->crashed() && !_was_crashed) {
        _was_crashed = true;
        Analyzer_NotCrashed_Result_CRASHED *result =
            new Analyzer_NotCrashed_Result_CRASHED();
        result->add_source(_data_sources.get("CRASH"));
        result->increase_severity_score(100);
        result->set_status(analyzer_status_fail);
        result->set_reason("Vehicle evaluated itself as crashed");
        result->set_T(_vehicle->T());
        add_result(result);
    } else if (!_vehicle->crashed() && _was_crashed) {
        _was_crashed = false;
    }
}

void Analyzer_NotCrashed::evaluate()
{
    evaluate_attitude();
    evaluate_vehicle();
}
