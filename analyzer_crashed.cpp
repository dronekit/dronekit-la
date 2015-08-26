#include "analyzer_crashed.h"

#include <syslog.h>
#include <stdio.h>

#include "util.h"

#include "analyzer_util.h"
#include "analyzervehicle_copter.h"


void Analyzer_Crashed::handle_decoded_message(uint64_t T, mavlink_param_value_t &param)
{
    evaluate(T);
}
void Analyzer_Crashed::handle_decoded_message(uint64_t T, mavlink_attitude_t &msg)
{
    Analyzer::handle_decoded_message(T, msg);
    evaluate(T);
    seen_packets_attitude = true;
}
void Analyzer_Crashed::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &servos)
{
    Analyzer::handle_decoded_message(T, servos);
    evaluate(T);
}



void Analyzer_Crashed::evaluate(uint64_t T)
{
    AnalyzerVehicle::Copter *v = (AnalyzerVehicle::Copter*&)_vehicle;

    if (v->exceeding_angle_max() &&
        v->any_motor_running_fast()) {
        crashed = true;
        crashed_timestamp = T;
        crashed_angle = (v->att().roll() > v->att().pitch()) ? v->att().roll() : v->att().pitch();
        for (uint8_t i=1; i<=v->_num_motors; i++) {
            crash_servo_output[i] = v->_servo_output[i];
        }
    }
}

void Analyzer_Crashed::results_json_results(Json::Value &root)
{
    Json::Value result(Json::objectValue);
    
    Json::Value reason(Json::arrayValue);
    uint8_t this_sin_score = 0;
    AnalyzerVehicle::Copter *v = (AnalyzerVehicle::Copter*&)_vehicle;
    if (crashed) {
        reason.append("Crashed");
        reason.append(string_format("ANGLE_MAX (%f > %f)", rad_to_deg(crashed_angle), v->param("ANGLE_MAX")/100)); // FIXME
        for (uint8_t i=1; i<=v->_num_motors; i++) {
            if (v->_servo_output[i] > v->is_flying_motor_threshold) {
                reason.append(string_format("SERVO_OUTPUT_RAW.servo%d_raw=%f",
                                            i, crash_servo_output[i]));
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
