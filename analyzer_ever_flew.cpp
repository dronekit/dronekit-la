#include "analyzer_ever_flew.h"

#include <syslog.h>
#include <stdio.h>

#include "util.h"

bool Analyzer_Ever_Flew::configure(INIReader *config) {
    if (!MAVLink_Message_Handler::configure(config)) {
	return false;
    }
    return true;
}

bool Analyzer_Ever_Flew::has_passed() {
    if (ever_armed == false) {
        return false;
    }
    if (servos_past_threshold == false) {
        return false;
    }
    return true;
}
// swiped from AnalyzeTest_Compass in experimental ArduPilot tree:
void Analyzer_Ever_Flew::handle_decoded_message(uint64_t T, mavlink_heartbeat_t &heartbeat)
{
    if (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
        ever_armed = true;
        if (has_passed()) {
            pass_timestamp = T;
        }
    }
}

void Analyzer_Ever_Flew::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &servos)
{
    uint16_t threshold = 1250;
    if (servos.servo1_raw > threshold ||
        servos.servo2_raw > threshold ||
        servos.servo3_raw > threshold ||
        servos.servo4_raw > threshold) {
        servos_past_threshold = true;
        if (has_passed()) {
            pass_timestamp = 1;
        }
    }
}

void Analyzer_Ever_Flew::results_json_results(Json::Value &results)
{
    Json::Value everflew(Json::objectValue);
    if (has_passed()) {
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
