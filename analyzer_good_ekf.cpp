#include "analyzer_good_ekf.h"

#include <syslog.h>
#include <stdio.h>
#include <unistd.h> // for fork()
#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Good_EKF::configure(INIReader *config) {
    if (!MAVLink_Message_Handler::configure(config)) {
	return false;
    }
    return true;
}

bool Analyzer_Good_EKF::has_failed() {
return (result_velocity_variance.max > velocity_variance.threshold_fail ||
        result_pos_horiz_variance.max > pos_horiz_variance.threshold_fail ||
        result_pos_vert_variance.max > pos_vert_variance.threshold_fail ||
        result_compass_variance.max > compass_variance.threshold_fail || 
        result_terrain_alt_variance.max > terrain_alt_variance.threshold_fail);
}

// swiped from AnalyzerTest_Compass in experimental ArduPilot tree:
void Analyzer_Good_EKF::handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &ekf_status_report)
{
    if (ekf_status_report.velocity_variance > result_velocity_variance.max) {
        result_velocity_variance.max = ekf_status_report.velocity_variance;
    }
    if (ekf_status_report.pos_horiz_variance > result_pos_horiz_variance.max) {
        result_pos_horiz_variance.max = ekf_status_report.pos_horiz_variance;
    }
    if (ekf_status_report.pos_vert_variance > result_pos_vert_variance.max) {
        result_pos_vert_variance.max = ekf_status_report.pos_vert_variance;
    }
    if (ekf_status_report.compass_variance > result_compass_variance.max) {
        result_compass_variance.max = ekf_status_report.compass_variance;
    }
    if (ekf_status_report.terrain_alt_variance > result_terrain_alt_variance.max) {
        result_terrain_alt_variance.max = ekf_status_report.terrain_alt_variance;
    }
    seen_ekf_packets = true;
}

extern char *examining_filename;

// void show_graph(const char *series)
// {
//     return;
//     if (fork()) { // parent
//         return;
//     }
//     const char *args[4];
//     args[0] = "mavgraph.py";
//     args[1] = examining_filename;
//     args[2] = series;
//     args[3] = NULL;
//     execvp(args[0], args);
//     ::fprintf(stderr, "exec of (%s) failed: %s\n", args[0], strerror(errno));
//     exit(1);
// }

void Analyzer_Good_EKF::results_json_results_do_variance(Json::Value &root, const struct ekf_variance variance, const struct ekf_variance_result variance_result)
{
    const char *name = variance.name;
    const double threshold_fail = variance.threshold_fail;
    const double threshold_warn = variance.threshold_warn;
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
        result["timestamp"] = (Json::UInt64)variance_result.T;
        root.append(result);
        // show_graph(tmp.c_str());
        add_evilness(this_sin_score);
    } else if (max > threshold_warn) {
        uint8_t this_sin_score = 4;
        reason.append(string_format("%s exceeds threshold (%f > %f)",
                                           name, max, threshold_fail));
        result["evilness"] = this_sin_score;
        result["reason"] = reason;
        result["status"] = "WARN";
        result["timestamp"] = (Json::UInt64)variance_result.T;
        root.append(result);
        // show_graph(tmp.c_str());
        add_evilness(this_sin_score);
    } else {
        reason.append(string_format("%s within threshold (%f <= %f)",
                                    name, max, threshold_fail));

        result["reason"] = reason;
        result["status"] = "OK";
        root.append(result);
    }
}
void Analyzer_Good_EKF::results_json_results(Json::Value &root)
{
    if (seen_ekf_packets) {
        results_json_results_do_variance(root, velocity_variance, result_velocity_variance);
        results_json_results_do_variance(root, pos_horiz_variance, result_pos_horiz_variance);
        results_json_results_do_variance(root, pos_vert_variance, result_pos_vert_variance);
        results_json_results_do_variance(root, compass_variance, result_compass_variance);
        results_json_results_do_variance(root, terrain_alt_variance, result_terrain_alt_variance);
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
