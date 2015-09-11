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

    _result.set_status(analyzer_status_fail);
    _result.set_reason("The vehicle never seemed to take off");
    
    return true;
}

void Analyzer_Ever_Flew::evaluate()
{
    if (_result.status() == analyzer_status_ok) {
        // already passed
        return;
    }

    if (_vehicle->is_armed()) {
        _result.set_ever_armed(true);
    }

    if (_vehicle->vehicletype() == AnalyzerVehicle::Base::vehicletype_t::copter) {
        if (((AnalyzerVehicle::Copter*&)_vehicle)->any_motor_running_fast()) {
            _result.set_servos_past_threshold(true);
        }
    }

    if (_vehicle->is_flying()) {
        _result.set_status(analyzer_status_ok);
        _result.set_reason("The vehicle appeared to fly");
        _result.set_pass_timestamp(_vehicle->T());
    }
}

void Analyzer_Ever_Flew::end_of_log(const uint32_t packet_count)
{
    if (_result.status() == analyzer_status_fail) {
        if (!_result.ever_armed()) {
            _result.add_evidence("Never Armed");
        }
        if (!_result.servos_past_threshold()) {
            _result.add_evidence("Servos never passed takeoff threshold");
        }
    }

    add_result(&_result);
}
