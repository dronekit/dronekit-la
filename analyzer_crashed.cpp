#include "analyzer_crashed.h"

#include <syslog.h>
#include <stdio.h>

#include "util.h"

#include "analyzer_util.h"

bool Analyzer_Crashed::configure(INIReader *config) {
    if (!MAVLink_Message_Handler::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Crashed::handle_decoded_message(uint64_t T, mavlink_param_value_t &param)
{
    const char *name = param.param_id;
    float value = param.param_value/100; // centi-degrees to degrees
    if (streq(name, "ANGLE_MAX")) {
        // convert to radians
        angle_max = value/180 * M_PI;
    }
}

bool Analyzer_Crashed::has_crashed()
{
    if (!exceeded_angle_max) {
        return false;
    }

    if (last_servo_output[1] < servo_output_threshold &&
        last_servo_output[2] < servo_output_threshold &&
        last_servo_output[3] < servo_output_threshold &&
        last_servo_output[4] < servo_output_threshold) {
        return false;
    }

    return true;
}

void Analyzer_Crashed::evaluate_crashed(uint64_t T)
{
    if (has_crashed()) {
        crashed = true;
        crashed_timestamp = T;
    }
}

void Analyzer_Crashed::handle_decoded_message(uint64_t T, mavlink_attitude_t &att)
{
    if (att.roll > angle_max || att.pitch > angle_max) {
        exceeded_angle_max = true;
        if (angle_max > angle_max_achieved) {
            angle_max_achieved = angle_max;
            exceeded_angle_max_timestamp = T;
        }
    }

    evaluate_crashed(T);
    seen_packets_attitude = true;
}

void Analyzer_Crashed::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &servos)
{
    last_servo_output[1] = servos.servo1_raw;
    last_servo_output[2] = servos.servo2_raw;
    last_servo_output[3] = servos.servo3_raw;
    last_servo_output[4] = servos.servo4_raw;

    evaluate_crashed(T);
}


void Analyzer_Crashed::results_json_results(Json::Value &root)
{
    Json::Value result(Json::objectValue);
    
    Json::Value reason(Json::arrayValue);
    uint8_t this_sin_score = 0;
    if (crashed) {
        reason.append("Crashed");
        reason.append(string_format("ANGLE_MAX (%f > %f)", angle_max_achieved*180/M_PI, angle_max*180/M_PI)); // FIXME
        for (uint8_t i=1; i<=4; i++) {
            if (last_servo_output[i] > servo_output_threshold) {
                reason.append(string_format("SERVO_OUTPUT_RAW.servo%d_raw=%f",
                                            i, last_servo_output[i]));
            }
        }
        this_sin_score = 10;
        Json::Value series(Json::arrayValue);
        series.append("PARAM");
        series.append("SERVO_OUTPUT_RAW.servo1_raw");
        series.append("SERVO_OUTPUT_RAW.servo2_raw");
        series.append("SERVO_OUTPUT_RAW.servo3_raw");
        series.append("SERVO_OUTPUT_RAW.servo4_raw");
        result["series"] = series;
        result["timestamp"] = (Json::UInt64)crashed_timestamp;
        result["status"] =  "FAIL";
        result["evilness"] = this_sin_score;
    } else if (!seen_packets_attitude) {
        reason.append("No attitude packets seen");
        result["status"] =  "WARN";
    } else {
        reason.append("Never Crashed");
        result["status"] =  "OK";
    }
    if (this_sin_score) {
        result["evilness"] = this_sin_score;
        add_evilness(this_sin_score);
    }
    result["reason"] = reason;

    root.append(result);
}
