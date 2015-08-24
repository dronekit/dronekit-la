#include "analyzer_ever_armed.h"

#include <syslog.h>
#include <stdio.h>

#include "util.h"

bool Analyzer_Ever_Armed::configure(INIReader *config) {
    if (!MAVLink_Message_Handler::configure(config)) {
	return false;
    }
    return true;
}

// swiped from AnalyzerTest_Compass in experimental ArduPilot tree:
void Analyzer_Ever_Armed::handle_decoded_message(uint64_t T, mavlink_heartbeat_t &heartbeat)
{
    if (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
        arm_time = T;
        ever_armed = true;
    }
}

const char *Analyzer_Ever_Armed::description()
{
    return "The vehicle armed";
}

void Analyzer_Ever_Armed::results_json_results(Json::Value &root)
{
    Json::Value result(Json::objectValue);
    
    Json::Value reason(Json::arrayValue);
    reason.append(ever_armed ? "Armed" : "Never Armed");
    result["reason"] = reason;
    if (ever_armed) {
        result["timestamp"] = (Json::UInt64)arm_time;
        result["status"] =  "OK";
    } else {
        Json::Value series(Json::arrayValue);
        series.append("HEARBEAT.base_mode & MAV_MODE_FLAG_SAFETY_ARMED");
        result["series"] = series;

        result["status"] =  "FAIL";
    }
    root.append(result);
}
