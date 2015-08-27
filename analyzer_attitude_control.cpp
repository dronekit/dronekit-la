#include "analyzer_attitude_control.h"

#include <stdio.h>
#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Attitude_Control::configure(INIReader *config) {
    if (!MAVLink_Message_Handler::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Attitude_Control::evaluate(uint64_t T)
{
    if (!_vehicle->is_flying()) {
        return;
    }
    float roll = _vehicle->att().roll();
    float desroll = _vehicle->nav().desroll();
    float roll_delta = fabs(roll - desroll);

    float pitch = _vehicle->att().pitch();
    float despitch = _vehicle->nav().despitch();
    float pitch_delta = fabs(pitch - despitch);

    float delta = (pitch_delta > roll_delta) ? pitch_delta : roll_delta;

// ::printf("%s: attitude control roll=%f desroll=%f (%f / %f)\n", timestamp, roll, desroll, delta, offset_fail);
    analyzer_status _status_prev = status();
    if (delta > offset_fail) {
        set_status(analyzer_status_fail);
    } else if (delta > offset_warn) {
        set_status(analyzer_status_warn);
    } else {
        set_status(analyzer_status_ok);
    }


    if (status() == _status_prev) {
        if (status() != analyzer_status_ok) {
            attitude_control_result &result = attitude_control_results[attitude_control_results_offset];
            if (delta > result.deltamax) {
                result.deltamax = delta;
                result.roll_at_deltamax = roll;
                result.desroll_at_deltamax = desroll;
                result.pitch_at_deltamax = pitch;
                result.despitch_at_deltamax = despitch;
            }
        }
    } else {
        attitude_control_results[attitude_control_results_offset].timestamp_stop = T;
        switch(_status_prev) {
        case analyzer_status_ok:
            break;
        case analyzer_status_warn:
        case analyzer_status_fail:
            attitude_control_results_offset++;
        }
        if (status() != analyzer_status_ok) {
            attitude_control_result &result = attitude_control_results[attitude_control_results_offset];
            result.status = status();
            result.timestamp_start = T;
            result.timestamp_stop = 0;
            result.deltamax = delta;
            result.roll_at_deltamax = roll;
            result.desroll_at_deltamax = desroll;
            result.pitch_at_deltamax = pitch;
            result.despitch_at_deltamax = despitch;
        }
        _status_prev = status();
    }
}

void Analyzer_Attitude_Control::end_of_log(uint32_t packet_count)
{
    if (status() != analyzer_status_ok) {
        // close off the final result
        attitude_control_results_offset++;
    }
}

void Analyzer_Attitude_Control::handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg)
{
    evaluate(T);
}

void Analyzer_Attitude_Control::handle_decoded_message(uint64_t T, mavlink_attitude_t &msg)
{
    evaluate(T);
}

void Analyzer_Attitude_Control::results_json_results(Json::Value &root)
{
    if (_vehicle->att().roll_modtime() > 0) {
        for (uint8_t i=0; i<attitude_control_results_offset; i++) {
            uint8_t this_sin_score = 10;
            Json::Value result(Json::objectValue);
            class attitude_control_result &x = attitude_control_results[i];
            result["status"] = x.status_as_string();
            result["evilness"] = this_sin_score;
            add_evilness(this_sin_score);
            result["reason"] = "Desired attitude not achieved";
            Json::Value evidence(Json::arrayValue);
            evidence.append(string_format("Max-Delta=%f", x.deltamax));
            evidence.append(string_format("Roll-at-Max-Delta=%f", x.roll_at_deltamax));
            evidence.append(string_format("Desired-Roll-at-Max-Delta=%f", x.desroll_at_deltamax));
            evidence.append(string_format("Pitch-at-Max-Delta=%f", x.pitch_at_deltamax));
            evidence.append(string_format("Desired-Pitch-at-Max-Delta=%f", x.despitch_at_deltamax));
            result["evidence"] = evidence;
            result["timestamp_start"] = (Json::UInt64)x.timestamp_start;
            if (x.timestamp_stop != 0) {
                result["timestamp_stop"] = (Json::UInt64)x.timestamp_stop;
            }

            // current structure makes this stuff awfully magical:
            // tomorrow this should change because we'll be updated from
            // df logs....
            Json::Value series(Json::arrayValue);
            series.append("NAV_CONTROLLER_OUTPUT.nav_roll");
            series.append("NAV_CONTROLLER_OUTPUT.nav_pitch");
            series.append("ATTITUDE.roll");
            series.append("ATTITUDE.pitch");
            result["series"] = series;

            root.append(result);
        }
    } else {
        uint8_t this_sin_score = 5;
        Json::Value result(Json::objectValue);
        // result["timestamp"] = 0;
        result["status"] = "WARN";
        Json::Value reason(Json::arrayValue);
        reason.append("Vehicle attitude never set");
        result["evilness"] = this_sin_score;
        add_evilness(this_sin_score);
        result["reason"] = reason;
        Json::Value series(Json::arrayValue);
        series.append("ATTITUDE.roll");
        series.append("ATTITUDE.pitch");
        result["series"] = series;
        root.append(result);
    }
}
