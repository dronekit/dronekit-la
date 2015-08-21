#include "analyzer_brownout.h"

#include <syslog.h>
#include <stdio.h>
#include <unistd.h> // for fork()
#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Brownout::configure(INIReader *config) {
    if (!MAVLink_Message_Handler::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Brownout::handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &vfr_hud)
{
    last_altitude = vfr_hud.alt;
    seen_packets = true;
}

void Analyzer_Brownout::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &servos)
{
    last_servo_output[1] = servos.servo1_raw;
    last_servo_output[2] = servos.servo2_raw;
    last_servo_output[3] = servos.servo3_raw;
    last_servo_output[4] = servos.servo4_raw;
}

void Analyzer_Brownout::results_json_results(Json::Value &root)
{
    if (seen_packets) {
        Json::Value result(Json::objectValue);
        result["timestamp"] = 0;
        Json::Value reason(Json::arrayValue);
        if (last_altitude > max_last_altitude &&
            (last_servo_output[1] > 1250 ||
             last_servo_output[2] > 1250 ||
             last_servo_output[3] > 1250 ||
             last_servo_output[4] > 1250
                )) {
            uint8_t this_sin_score = 10;
            result["status"] = "FAILED";
            result["evilness"] = this_sin_score;
            add_evilness(this_sin_score);
            reason.append(string_format("Possible brownout detected (end of log at %f metres, servo outputs (%f/%f/%f/%f))",
                                        last_altitude, last_servo_output[1], last_servo_output[2], last_servo_output[3], last_servo_output[4] ));
        } else {
            result["status"] = "OK";
            reason.append(string_format("No brownout detected (final altitude %f metres", last_altitude));
        }
        result["reason"] = reason;
        root.append(result);
    } else {
        uint8_t this_sin_score = 5;
        Json::Value result(Json::objectValue);
        // result["timestamp"] = 0;
        result["status"] = "WARN";
        Json::Value reason(Json::arrayValue);
        reason.append("No VFR_HUD messages received");
        result["evilness"] = this_sin_score;
        add_evilness(this_sin_score);
        result["reason"] = reason;
        Json::Value series(Json::arrayValue);
        series.append("VFR_HUD.alt");
        series.append("SERVO_OUTPUT_RAW.servo1_raw");
        series.append("SERVO_OUTPUT_RAW.servo2_raw");
        series.append("SERVO_OUTPUT_RAW.servo3_raw");
        series.append("SERVO_OUTPUT_RAW.servo4_raw");
        result["series"] = series;
        root.append(result);
    }
}
