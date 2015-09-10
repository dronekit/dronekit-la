#include "analyzer_ever_flew.h"

#include "util.h"

void Analyzer_Ever_Flew::evaluate()
{
    if (pass_timestamp != 0) {
        // already passed
        return;
    }

    if (_vehicle->is_armed()) {
        ever_armed = true;
    }
    if (_vehicle->vehicletype() == AnalyzerVehicle::Base::vehicletype_t::copter) {
        if (((AnalyzerVehicle::Copter*&)_vehicle)->any_motor_running_fast()) {
            servos_past_threshold = true;
        }
    }

    if (_vehicle->is_flying()) {
        pass_timestamp = _vehicle->T();
    }
}

void Analyzer_Ever_Flew::results_json_results(Json::Value &results)
{
    Json::Value everflew(Json::objectValue);
    if (pass_timestamp) {
        Json::Value timestamp(Json::objectValue);
        timestamp = (Json::UInt64)pass_timestamp;
        everflew["timestamp"] = timestamp;
        everflew["status"] = "OK";
        everflew["reason"] = "The vehicle appeared to fly";
    } else {
        everflew["status"] = "FAIL";
        Json::Value evidence(Json::arrayValue);
        if (!ever_armed) {
            evidence.append("Never Armed");
        }
        if (!servos_past_threshold) {
            evidence.append("Servos never passed takeoff threshold");
        }
        everflew["evidence"] = evidence;
        everflew["reason"] = "The vehicle never seemed to take off";
    }

    results.append(everflew);
}
