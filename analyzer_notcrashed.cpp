#include "analyzer_notcrashed.h"

#include "util.h"

#include "analyzer_util.h"
#include "analyzervehicle_copter.h"


void Analyzer_NotCrashed::handle_decoded_message(uint64_t T, mavlink_param_value_t &param)
{
    evaluate(T);
}
void Analyzer_NotCrashed::handle_decoded_message(uint64_t T, mavlink_attitude_t &msg)
{
    Analyzer::handle_decoded_message(T, msg);
    evaluate(T);
    seen_packets_attitude = true;
}
void Analyzer_NotCrashed::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &servos)
{
    Analyzer::handle_decoded_message(T, servos);
    evaluate(T);
}



void Analyzer_NotCrashed::end_of_log(const uint32_t packet_count)
{
    if (status() != analyzer_status_ok) {
        // close off the final result
        notcrashed_results_offset++;
    }
}

void Analyzer_NotCrashed::evaluate(uint64_t T)
{
    AnalyzerVehicle::Copter *v = (AnalyzerVehicle::Copter*&)_vehicle;

    analyzer_status _status_prev = status();
    if (v->exceeding_angle_max() &&
        v->any_motor_running_fast()) {
        set_status(analyzer_status_fail);
    } else {
        set_status(analyzer_status_ok);
    }


    if (status() == _status_prev) {
        // nothing... yet...
    } else {
        switch(_status_prev) {
        case analyzer_status_ok:
            break;
        case analyzer_status_warn:
        case analyzer_status_fail:
            if (status() != analyzer_status_ok) {
                // only "crash" if this state persists for >1second:
                if (T - notcrashed_results[notcrashed_results_offset].timestamp_start > 1000000) {
                    // accept this result
                    notcrashed_results[notcrashed_results_offset].timestamp_stop = T;
                    notcrashed_results_offset++;
                }
            }
        }
        if (status() != analyzer_status_ok) {
            notcrashed_result &result = notcrashed_results[notcrashed_results_offset];
            result.status = status();
            result.timestamp_start = T;
            result.timestamp_stop = 0;
            result.angle = (v->att().roll() > v->att().pitch()) ? v->att().roll() : v->att().pitch();
            result.angle_max = v->param("ANGLE_MAX")/100;
            for (uint8_t i=1; i<=v->_num_motors; i++) {
                result.servo_output[i] = v->_servo_output[i];
            }
        }
        _status_prev = status();
    }
}

void Analyzer_NotCrashed::add_series(Json::Value &root)
{
    Json::Value series(Json::arrayValue);
    series.append("PARAM");
    series.append("SERVO_OUTPUT_RAW.servo1_raw");
    series.append("SERVO_OUTPUT_RAW.servo2_raw");
    series.append("SERVO_OUTPUT_RAW.servo3_raw");
    series.append("SERVO_OUTPUT_RAW.servo4_raw");
    root["series"] = series;
}

void Analyzer_NotCrashed::results_json_results(Json::Value &root)
{
    AnalyzerVehicle::Copter *v = (AnalyzerVehicle::Copter*&)_vehicle;
    for (uint8_t i=0; i<notcrashed_results_offset; i++) {
        class notcrashed_result &x = notcrashed_results[i];

        Json::Value result(Json::objectValue);
    
        result["status"] = x.status_as_string();
        result["reason"] = "Vehicle is past maximum allowed angle and running its motors";
        Json::Value evidence(Json::arrayValue);
        evidence.append(string_format("ANGLE_MAX (%f > %f)", x.angle, x.angle_max));
        for (uint8_t i=1; i<=v->_num_motors; i++) {
            if (v->_servo_output[i] > v->is_flying_motor_threshold) {
                evidence.append(string_format("SERVO_OUTPUT_RAW.servo%d_raw=%f",
                                              i, x.servo_output[i]));
            }
        }
        result["evidence"] = evidence;

        result["timestamp_start"] = (Json::UInt64)x.timestamp_start;
        if (x.timestamp_stop != 0) {
            result["timestamp_stop"] = (Json::UInt64)x.timestamp_stop;
        }

        add_series(result);

        const uint8_t this_sin_score = 10;
        result["evilness"] = this_sin_score;
        add_evilness(this_sin_score);
        root.append(result);
    }

    if (notcrashed_results_offset == 0) {
        Json::Value result(Json::objectValue);
        if (_vehicle->att().roll_modtime() == 0) {
            result["reason"] = "Vehicle's attitude never updated";
            result["status"] =  "WARN";
        } else {
            result["reason"] = "Never Crashed";
            result["status"] =  "OK";
        }
        add_series(result);
        root.append(result);
    }

}
