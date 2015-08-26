#include "analyzer_ever_flew.h"

#include <syslog.h>
#include <stdio.h>

#include "util.h"

void Analyzer_Ever_Flew::evaluate(uint64_t T)
{
    if (pass_timestamp != 0) {
        // already passed
        return;
    }

    if (_vehicle->is_armed()) {
        ever_armed = true;
    }
    if (((AnalyzerVehicle::Copter*&)_vehicle)->any_motor_running_fast()) {
        servos_past_threshold = true;
    }

    if (_vehicle->is_flying()) {
        pass_timestamp = T;
    }
}

void Analyzer_Ever_Flew::handle_decoded_message(uint64_t T, mavlink_heartbeat_t &heartbeat)
{
    evaluate(T);
}

void Analyzer_Ever_Flew::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &servos)
{
    evaluate(T);
}

void Analyzer_Ever_Flew::results_json_results(Json::Value &results)
{
    Json::Value everflew(Json::objectValue);
    if (pass_timestamp) {
        Json::Value timestamp(Json::objectValue);
        timestamp = (Json::UInt64)pass_timestamp;
        everflew["timestamp"] = timestamp;
        everflew["status"] = "OK";
    } else {
        everflew["status"] = "FAIL";
        Json::Value reason(Json::arrayValue);
        if (!ever_armed) {
            Json::Value never_armed(Json::objectValue);
            reason.append("Never Armed");
        }
        if (!servos_past_threshold) {
            Json::Value never_armed(Json::objectValue);
            reason.append("Servos never passed takeoff threshold");
        }
        everflew["reason"] = reason;
    }

    results.append(everflew);
}
