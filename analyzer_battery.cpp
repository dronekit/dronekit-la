#include "analyzer_battery.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Battery::configure(INIReader *config) {
    return true;
}

void Analyzer_Battery::evaluate()
{
    if (_vehicle->battery_remaining_T() &&
        _vehicle->battery_remaining() < lowest_battery_remaining_seen) {
        lowest_battery_remaining_seen = _vehicle->battery_remaining();
        lowest_battery_remaining_seen_T = _vehicle->T();
    }
    if (_vehicle->battery_in_failsafe()) {
        seen_failsafe_battery_event = true;
        seen_failsafe_battery_event_T = _vehicle->battery_in_failsafe_T();
    }
}

void Analyzer_Battery::results_json_results(Json::Value &root)
{
    if (lowest_battery_remaining_seen_T) {
        Json::Value result(Json::objectValue);
        result["timestamp"] = (Json::UInt64)lowest_battery_remaining_seen_T;
        if (lowest_battery_remaining_seen < low_battery_threshold) {
            result["status"] = "FAILED";
            result["reason"] = "Battery fell below failsafe threshold";

            Json::Value evidence(Json::arrayValue);
            evidence.append(string_format("Battery below failsafe (%.0f%% < %.0f%%)",
                                          lowest_battery_remaining_seen, low_battery_threshold));
            result["evidence"] = evidence;

            Json::Value series(Json::arrayValue);
            series.append("SYS_STATUS.battery_remaining");
            result["series"] = series;
        } else {
            result["status"] = "OK";
            result["reason"] = "Battery never below failsafe";

            Json::Value evidence(Json::arrayValue);
            evidence.append(string_format("battery-remaining=%f%%", lowest_battery_remaining_seen));
            evidence.append(string_format("failsafe=%f%%", low_battery_threshold));
            result["evidence"] = evidence;
        }
        root.append(result);
    }
    if (seen_failsafe_battery_event_T) {
        Json::Value result(Json::objectValue);
        result["timestamp"] = (Json::UInt64)seen_failsafe_battery_event_T;
        result["reason"] = "Battery failsafe event received";
        Json::Value evidence(Json::arrayValue);
        evidence.append(string_format("Failsafe set at %u",
                                      seen_failsafe_battery_event_T));
        result["evidence"] = evidence;

        Json::Value series(Json::arrayValue);
        series.append("EV");
        result["series"] = series;
        root.append(result);
    }

    // else {
    //     Json::Value result(Json::objectValue);
    //     result["timestamp"] = 0;
    //     result["status"] = "WARN";
    //     Json::Value reason(Json::arrayValue);
    //     reason.append("No SYS_STATUS messages received");
    //     result["reason"] = reason;
    //     root.append(result);
    // }
}
