#include "analyzer_battery.h"

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

void Analyzer_Battery::handle_decoded_message(uint64_t T, mavlink_sys_status_t &sys_status)
{
    if (sys_status.battery_remaining < lowest_battery_remaining_seen) {
        lowest_battery_remaining_seen = sys_status.battery_remaining;
    }
    seen_sys_status_packets = true;
}

void Analyzer_Battery::results_json_results(Json::Value &root)
{
    Json::Value result(Json::objectValue);
    if (seen_sys_status_packets) {
        result["timestamp"] = 0;
        if (has_failed()) {
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
    } else {
        result["timestamp"] = 0;
        result["status"] = "WARN";
        Json::Value reason(Json::arrayValue);
        reason.append("No SYS_STATUS messages received");
        result["reason"] = reason;
    }
    root.append(result);
}
