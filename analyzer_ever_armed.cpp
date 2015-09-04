#include "analyzer_ever_armed.h"

#include "util.h"

void Analyzer_Ever_Armed::evaluate()
{
    if (_vehicle->is_armed()) {
        arm_time = _vehicle->T();
        ever_armed = true;
    }
}

void Analyzer_Ever_Armed::results_json_results(Json::Value &root)
{
    Json::Value result(Json::objectValue);
    
    result["reason"] = ever_armed ? "The vehicle armed" : "The vehicle never armed";
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
