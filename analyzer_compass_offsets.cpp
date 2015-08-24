#include "analyzer_compass_offsets.h"

#include <syslog.h>
#include <stdio.h>

#include "util.h"

bool Analyzer_Compass_Offsets::configure(INIReader *config) {
    if (!MAVLink_Message_Handler::configure(config)) {
	return false;
    }
    return true;
}

// void Analyzer_Compass_Offsets::handle_decoded_message(mavlink_param_value_t &param)
// {
//     ::printf("Param found\n");
// }

double vec_len(double vec[3]) {
    return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

// swiped from AnalyzeTest_Compass in experimental ArduPilot tree:
void Analyzer_Compass_Offsets::handle_decoded_message(uint64_t T, mavlink_param_value_t &param)
{
    int8_t ofs = -1;
    const char *name = param.param_id;
    const uint8_t namelen = MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN;
    float value = param.param_value;

    if (compass_offset_results_offset == MAX_COMPASS_OFFSET_RESULTS) {
        compass_offset_results_overrun = true;
        return;
    }
    // watch the following; name is not null-terminated when 16-bytes long ;-)
    // ::printf("Param: %s = %f\n", name, value);

    if (!strncmp(name, "COMPASS_OFS_X", namelen)) {
        ofs = 0;
    } else if (!strncmp(name, "COMPASS_OFS_Y", namelen)) {
        ofs = 1;
    } else if (!strncmp(name, "COMPASS_OFS_Z", namelen)) {
        ofs = 2;
    }
    if (ofs != -1) {
        if (have_compass_ofs[ofs] &&
            compass_ofs[ofs] == value) {
            // skip duplicate values
            return;
        }
        compass_ofs[ofs] = value;
        have_compass_ofs[ofs] = true;
        if (have_compass_ofs[0] &&
            have_compass_ofs[1] &&
            have_compass_ofs[2]) {
            double len = vec_len(compass_ofs);
            compass_offset_status status;
            if (len >= fail_offset) {
                status = compass_offset_fail;
                // ::printf("FAIL: COMPASS_OFS %f>%d\n", len, fail_offset);
            } else if (len >= warn_offset) {
                // ::printf("WARNING: COMPASS_OFS %f>%d\n", len, warn_offset);
                status = compass_offset_warn;
            } else if (is_zero(len)) {
                // ::printf("WARNING: Zero COMPASS_OFS - that's probably bad\n");
                status = compass_offset_zero;
            } else {
                // ::printf("OK: COMPASS_OFS %f looks good\n", len);
                status = compass_offset_ok;
            }

            compass_offset_results[compass_offset_results_offset].len = len;
            compass_offset_results[compass_offset_results_offset].status = status;
            compass_offset_results[compass_offset_results_offset].timestamp = T;
            compass_offset_results_offset++;
            // have_compass_ofs[0] = false;
            // have_compass_ofs[1] = false;
            // have_compass_ofs[2] = false;
        }
    }
}

#include <stdlib.h>

const char * Analyzer_Compass_Offsets::results_json_compass_offsets_status_string(compass_offset_result result)
{
    switch(result.status) {
    case compass_offset_warn:
        return "WARNING";
    case compass_offset_ok:
        return "OK";
    case compass_offset_zero:
        return "WARNING";
    case compass_offset_fail:
        return "FAIL";
    default:
        return "Unknown";
    }
}

void Analyzer_Compass_Offsets::addStatusReason(Json::Value &root,compass_offset_result result)
{
    std::string reason  = std::string("");
    reason += "COMPASS_OFS " + to_string(result.len);
    switch(result.status) {
    case compass_offset_warn:
        reason += " > ";
        reason += to_string(warn_offset);
        break;
    case compass_offset_ok:
        reason += " <= ";
        reason += to_string(warn_offset);
        break;
    case compass_offset_zero:
        reason += " == 0";
        break;
    case compass_offset_fail:
        reason += " > ";
        reason += to_string(fail_offset);
        break;
    }
    root["reason"] = reason;
}

void Analyzer_Compass_Offsets::do_add_evilness(struct compass_offset_result result)
{
    switch(result.status) {
    case compass_offset_warn:
        add_evilness(2);
        break;
    case compass_offset_fail:
        add_evilness(5);
        break;
    case compass_offset_zero:
        add_evilness(4);
        break;
    case compass_offset_ok:
        break;
    }
}

void Analyzer_Compass_Offsets::results_json_compass_offsets(Json::Value &root)
{
    Json::Value ret(Json::arrayValue);
    for (uint8_t i=0; i < compass_offset_results_offset; i++) {
        Json::Value result(Json::objectValue);
        result["timestamp"] = (Json::UInt64)(compass_offset_results[i].timestamp);
        result["status"] = results_json_compass_offsets_status_string(compass_offset_results[i]);
        do_add_evilness(compass_offset_results[i]);
        Json::Value series(Json::arrayValue);
        series.append("PARM");
        result["series"] = series;
        addStatusReason(result,compass_offset_results[i]);
        root.append(result);
    }
    if (compass_offset_results_offset == 0) {
        Json::Value result(Json::objectValue);
        result["status"] = "FAIL";
        Json::Value series(Json::arrayValue);
        series.append("PARM");
        result["series"] = series;
        result["reason"] = "No compass offset parameter set seen";
        root.append(result);
        add_evilness(5);
    }
}


void Analyzer_Compass_Offsets::results_json_results(Json::Value &root)
{
    results_json_compass_offsets(root);
}
