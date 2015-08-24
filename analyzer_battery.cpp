#include "analyzer_battery.h"

#include <syslog.h>
#include <stdio.h>
#include <unistd.h> // for fork()
#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Battery::configure(INIReader *config) {
    if (!MAVLink_Message_Handler::configure(config)) {
	return false;
    }
    return true;
}

bool Analyzer_Battery::has_failed() {
    return (lowest_battery_remaining_seen < low_battery_threshold);
}

// swiped from AnalyzerTest_Compass in experimental ArduPilot tree:
void Analyzer_Battery::handle_decoded_message(uint64_t T, mavlink_sys_status_t &sys_status)
{
    if (sys_status.battery_remaining < lowest_battery_remaining_seen) {
        lowest_battery_remaining_seen = sys_status.battery_remaining;
    }
    seen_sys_status_packets = true;
}

void Analyzer_Battery::results_json_results(Json::Value &root)
{
    if (seen_sys_status_packets) {
        Json::Value result(Json::objectValue);
        result["timestamp"] = 0;
        Json::Value reason(Json::arrayValue);
        if (has_failed()) {
            result["status"] = "FAILED";
            reason.append(string_format("Battery below failsafe (%f < %f)",
                                        lowest_battery_remaining_seen, low_battery_threshold));
            Json::Value series(Json::arrayValue);
            series.append("SYS_STATUS.battery_remaining");
            result["series"] = series;
        } else {
            result["status"] = "OK";
            reason.append(string_format("Battery never below failsafe (%f >= %f)",
                                        lowest_battery_remaining_seen, low_battery_threshold));
        }
        result["reason"] = reason;
        root.append(result);
    } else {
        Json::Value result(Json::objectValue);
        result["timestamp"] = 0;
        result["status"] = "WARN";
        Json::Value reason(Json::arrayValue);
        reason.append("No SYS_STATUS messages received");
        result["reason"] = reason;
        root.append(result);
    }
}
