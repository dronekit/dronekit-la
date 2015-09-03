#include "analyzer_good_ekf.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Good_EKF::configure(INIReader *config) {
    if (!MAVLink_Message_Handler::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Good_EKF::close_variance_result(struct ekf_variance_result &result)
{
    result_variance[next_result_variance].variance = result.variance;
    result_variance[next_result_variance].T_start = result.T_start;
    result_variance[next_result_variance].T_stop = result.T_stop;
    result_variance[next_result_variance].max = result.max;
    next_result_variance++;
}

void Analyzer_Good_EKF::maybe_close_variance_result(struct ekf_variance_result &result)
{
    if (result.T_start != 0) {
        close_variance_result(result);
    }
}
void Analyzer_Good_EKF::end_of_log(uint32_t packet_count)
{
    // almost like we need a sub-object to feed state to :-/
    maybe_close_variance_result(result_velocity_variance);
    maybe_close_variance_result(result_pos_horiz_variance);
    maybe_close_variance_result(result_pos_vert_variance);
    maybe_close_variance_result(result_compass_variance);
    maybe_close_variance_result(result_terrain_alt_variance);
}

void Analyzer_Good_EKF::handle_variance(uint64_t T,
                                        struct ekf_variance &variance,
                                        struct ekf_variance_result &result,
                                        double value)
{
    if (result.T_start) {
        if (value > result.max) {
            // a new record!
            result.max = value;
        } else if (value > variance.threshold_warn) {
            // we continue to exceed variances...
        } else {
            // our variances have come down sufficiently to close this instance
            result.T_stop = T;
            close_variance_result(result);
            result.T_start = 0;
        }
    } else {
        if (value > variance.threshold_warn) {
            // we have exceeeded a threshold
            result.T_start = T;
            result.max = value;
        }
    }
}

// swiped from AnalyzerTest_Compass in experimental ArduPilot tree:
void Analyzer_Good_EKF::handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &ekf_status_report)
{
    handle_variance(T,
                    velocity_variance,
                    result_velocity_variance,
                    ekf_status_report.velocity_variance);
     handle_variance(T,
                    pos_horiz_variance,
                    result_pos_horiz_variance,
                    ekf_status_report.pos_horiz_variance);
     handle_variance(T,
                     pos_vert_variance,
                     result_pos_vert_variance,
                     ekf_status_report.pos_vert_variance);
     handle_variance(T,
                    compass_variance,
                    result_compass_variance,
                    ekf_status_report.compass_variance);
     handle_variance(T,
                    terrain_alt_variance,
                    result_terrain_alt_variance,
                    ekf_status_report.terrain_alt_variance);

    seen_ekf_packets = true;
}

void Analyzer_Good_EKF::results_json_results_do_variance(Json::Value &root, const struct ekf_variance_result variance_result)
{
    const struct ekf_variance *variance  = variance_result.variance;
    const char *name = variance->name;
    const double threshold_fail = variance->threshold_fail;
    const double threshold_warn = variance->threshold_warn;
    const double max = variance_result.max;
    
    Json::Value result(Json::objectValue);
    std::string tmp = string_format("%s.%s", "EKF_STATUS_REPORT", name);
    result["series"] = tmp;
    Json::Value reason(Json::arrayValue);
    if (max > threshold_fail) {
        uint8_t this_sin_score = 10;
        reason.append(string_format("%s exceeds threshold (%f > %f)",
                                    name, max, threshold_fail));
        result["reason"] = reason;
        result["status"] = "FAIL";
        result["evilness"] = this_sin_score;
        result["timestamp_start"] = (Json::UInt64)variance_result.T_start;
        result["timestamp_stop"] = (Json::UInt64)variance_result.T_stop;
        root.append(result);
        add_evilness(this_sin_score);
    } else if (max > threshold_warn) {
        uint8_t this_sin_score = 4;
        reason.append(string_format("%s exceeds threshold (%f > %f)",
                                           name, max, threshold_warn));
        result["evilness"] = this_sin_score;
        result["reason"] = reason;
        result["status"] = "WARN";
        result["timestamp_start"] = (Json::UInt64)variance_result.T_start;
        result["timestamp_stop"] = (Json::UInt64)variance_result.T_stop;
        root.append(result);
        add_evilness(this_sin_score);
    }
}
void Analyzer_Good_EKF::results_json_results(Json::Value &root)
{
    if (seen_ekf_packets) {
        for (uint8_t i=0; i<next_result_variance;i++) {
            results_json_results_do_variance(root, result_variance[i]);
        }
    } else {
        Json::Value result(Json::objectValue);
        result["timestamp"] = 0;
        result["status"] = "WARN";
        Json::Value reason(Json::arrayValue);
        reason.append("No EKF_STATUS_REPORT messages received");
        result["reason"] = reason;
        root.append(result);
    }
}
