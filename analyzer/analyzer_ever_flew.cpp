#include "analyzer_ever_flew.h"

#include "util.h"

void Analyzer_Ever_Flew_Result::to_json(Json::Value &root) const
{
    Analyzer_Result_Summary::to_json(root);
    root["timestamp-first-flying"] = (Json::UInt64)pass_timestamp();
}

bool Analyzer_Ever_Flew::configure(INIReader *config)
{
    if (!Analyzer::configure(config)) {
	return false;
    }

    return true;
}

void Analyzer_Ever_Flew::evaluate()
{
    bool is_flying = _vehicle->is_flying();
    if (is_flying != _was_flying) {
        if (is_flying) {
            // started flying
            _fly_start_time = _vehicle->T();
        } else {
            // stopped flying
            _total_flight_time += _vehicle->T() - _fly_start_time;
        }
        _was_flying = is_flying;
    }

    if (_result.status() == analyzer_status_ok) {
        // already passed
        return;
    }

    if (_vehicle->is_armed()) {
        _result.set_ever_armed(true);
    }

    if (_vehicle->vehicletype() == AnalyzerVehicle::Base::vehicletype_t::copter) {
        if (!_added_servo_output) {
            _result.add_source(_data_sources.get("SERVO_OUTPUT"));
            _added_servo_output = true;
        }

        if (((AnalyzerVehicle::Copter*&)_vehicle)->any_motor_running_fast()) {
            _result.set_servos_past_threshold(true);
        }
    }

    if (is_flying) {
        _result.set_status(analyzer_status_ok);
        _result.set_reason("The vehicle appeared to fly");
        _result.set_pass_timestamp(_vehicle->T());
    }
}

void Analyzer_Ever_Flew::end_of_log(const uint32_t packet_count UNUSED)
{
    if (_vehicle->is_flying()) {
        _total_flight_time += _vehicle->T() - _fly_start_time;
    }
    if (_result.status() == analyzer_status_fail) {
        if (!_result.ever_armed()) {
            _result.add_evidence("Never Armed");
        }
        switch (_vehicle->vehicletype()) {
        case AnalyzerVehicle::Base::vehicletype_t::copter:
            if (!_result.servos_past_threshold()) {
                _result.add_evidence("Servos never passed takeoff threshold");
            }
            break;
        case AnalyzerVehicle::Base::vehicletype_t::plane:
            break;
        case AnalyzerVehicle::Base::vehicletype_t::invalid:
            _result.add_evidence("Vehicle type was never defined");
            break;
        }
    }

    add_result(&_result);
}
