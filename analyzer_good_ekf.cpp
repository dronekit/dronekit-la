#include "analyzer_good_ekf.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Good_EKF::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Good_EKF::close_variance_result(struct ekf_variance_result *result)
{
    result_variance[next_result_variance].variance = result->variance;
    result_variance[next_result_variance].T_start = result->T_start;
    result_variance[next_result_variance].T_stop = result->T_stop;
    result_variance[next_result_variance].max = result->max;
    next_result_variance++;
}

void Analyzer_Good_EKF::end_of_log(uint32_t packet_count)
{
    std::map<const std::string, double> variances = _vehicle->ekf_variances();
    
    for (std::map<const std::string, double>::const_iterator it = variances.begin();
         it != variances.end();
         ++it) {
        // ::fprintf(stderr, "would evaluate (%s)\n", (*it).first.c_str());
        std::string name = (*it).first;
        struct ekf_variance_result *result = variance_result(name);
        if (result->T_start != 0) {
            close_variance_result(result);
        }
    }
}

bool Analyzer_Good_EKF::ekf_flags_bad(uint16_t flags)
{
    for (uint16_t i=1; i<EKF_STATUS_FLAGS_ENUM_END; i<<=1) {
        if (!(flags & i)) {
            return true;
        }
    }
    return false;
}

void Analyzer_Good_EKF::evaluate_variance(std::string name, double value)
{
    struct ekf_variance_result *result = variance_result(name);
    if (result ==NULL) {
        ::fprintf(stderr, "result for (%s) is NULL\n", name.c_str());
        abort();
    }
    struct ekf_variance *variance = result->variance;

    if (result->T_start) {
        if (value > result->max) {
            // a new record!
            result->max = value;
        } else if (value > variance->threshold_warn) {
            // we continue to exceed variances...
        } else {
            // our variances have come down sufficiently to close this instance
            result->T_stop = _vehicle->T();
            close_variance_result(result);
            result->T_start = 0;
        }
    } else {
        if (value > variance->threshold_warn) {
            // we have exceeeded a threshold
            result->T_start = _vehicle->T();
            result->max = value;
        }
    }
}

void Analyzer_Good_EKF::evaluate_variances()
{
    std::map<const std::string, double> variances = _vehicle->ekf_variances();
    
    for (std::map<const std::string, double>::const_iterator it = variances.begin();
         it != variances.end();
         ++it) {
        evaluate_variance((*it).first, (*it).second);
    }
}

// FIXME: should be taking timestamps from ekf_flags_t rather than
// _vehicle->T()
void Analyzer_Good_EKF::evaluate_flags()
{
    // ::fprintf(stderr, "flags: %d\n", _vehicle->ekf_flags());
    if (next_result_flags >= MAX_FLAGS_RESULTS) {
        return;
    }
    if (result_flags[next_result_flags].T_start) {
        if (result_flags[next_result_flags].flags != _vehicle->ekf_flags()) {
            // close off the old one
            result_flags[next_result_flags].T_stop = _vehicle->T();
            next_result_flags++;
            if (next_result_flags >= MAX_FLAGS_RESULTS) {
                return;
            }
            if (ekf_flags_bad(_vehicle->ekf_flags())) {
                result_flags[next_result_flags].T_start = _vehicle->T();
                result_flags[next_result_flags].flags = _vehicle->ekf_flags();
            }
        }
    } else {
        if (ekf_flags_bad(_vehicle->ekf_flags())) {
            result_flags[next_result_flags].T_start = _vehicle->T();
            result_flags[next_result_flags].flags = _vehicle->ekf_flags();
        }
    }
}


void Analyzer_Good_EKF::evaluate()
{
    evaluate_variances();
    evaluate_flags();
}
    

void Analyzer_Good_EKF::results_json_results_do_variance(Json::Value &root, const struct Analyzer_Good_EKF::ekf_variance_result *variance_result)
{
    struct ekf_variance *variance  = variance_result->variance;
    const char *name = variance->name;
    const double threshold_fail = variance->threshold_fail;
    const double threshold_warn = variance->threshold_warn;
    const double max = variance_result->max;
    
    Json::Value result(Json::objectValue);
    std::string tmp = string_format("%s.%s", "EKF_STATUS_REPORT", name);
    result["series"] = tmp;
    if (max > threshold_fail) {
        uint8_t this_sin_score = 10;
        result["reason"] = string_format("%s exceeds fail threshold", name);

        Json::Value evidence(Json::arrayValue);
        evidence.append(string_format("max-variance=%f", max));
        evidence.append(string_format("threshold=%f", threshold_fail));
        result["evidence"] = evidence;

        result["status"] = "FAIL";
        result["severity-score"] = this_sin_score;
        result["evilness"] = result["severity-score"];
        result["timestamp_start"] = (Json::UInt64)variance_result->T_start;
        result["timestamp_stop"] = (Json::UInt64)variance_result->T_stop;
        root.append(result);
        add_severity_score(this_sin_score);
    } else if (max > threshold_warn) {
        uint8_t this_sin_score = 4;
        result["reason"] = string_format("%s exceeds warn threshold", name);

        Json::Value evidence(Json::arrayValue);
        evidence.append(string_format("max=%f", max));
        evidence.append(string_format("threshold=%f", threshold_warn));
        result["evidence"] = evidence;

        result["severity-score"] = this_sin_score;
        result["evilness"] = result["severity-score"];
        result["status"] = "WARN";
        result["timestamp_start"] = (Json::UInt64)variance_result->T_start;
        result["timestamp_stop"] = (Json::UInt64)variance_result->T_stop;
        root.append(result);
        add_severity_score(this_sin_score);
    }
}

void Analyzer_Good_EKF::results_json_results_do_flags(Json::Value &root, const struct ekf_flags_result flags_result)
{
    
    Json::Value result(Json::objectValue);
    std::string tmp = string_format("%s.%s", "EKF_STATUS_REPORT", "flags");
    result["series"] = tmp;
    result["reason"] = "The EKF status report indicates a problem with the EKF";
    Json::Value evidence(Json::arrayValue);

    uint8_t this_sin_score = 10;
    if (!(flags_result.flags & EKF_ATTITUDE)) {
        this_sin_score++;
        evidence.append("attitude estimate bad");
    }
    if (!(flags_result.flags & EKF_VELOCITY_HORIZ)) {
        this_sin_score++;
        evidence.append("horizontal velocity estimate bad");
    }
    if (!(flags_result.flags & EKF_VELOCITY_VERT)) {
        this_sin_score++;
        evidence.append("vertical velocity estimate bad");
    }
    if (!(flags_result.flags & EKF_POS_HORIZ_REL)) {
        this_sin_score++;
        evidence.append("horizontal position (relative) estimate bad");
    }
    if (!(flags_result.flags & EKF_POS_HORIZ_ABS)) {
        this_sin_score++;
        evidence.append("horizontal position (absolute) estimate bad");
    }
    if (!(flags_result.flags & EKF_POS_VERT_ABS)) {
        this_sin_score++;
        evidence.append("vertical position (absolute) estimate bad");
    }
    if (!(flags_result.flags & EKF_POS_VERT_AGL)) {
        this_sin_score++;
        evidence.append("vertical position (above ground) estimate bad");
    }
    if (!(flags_result.flags & EKF_CONST_POS_MODE)) {
        this_sin_score++;
        evidence.append("In constant position mode (no abs or rel position)");
    }
    if (!(flags_result.flags & EKF_PRED_POS_HORIZ_REL)) {
        this_sin_score++;
        evidence.append("Predicted horizontal position (relative) bad");
    }
        if (!(flags_result.flags & EKF_PRED_POS_HORIZ_ABS)) {
        this_sin_score++;
        evidence.append("Predicted horizontal position (absolute) bad");
    }

    result["evidence"] = evidence;
    result["status"] = "FAIL";
    result["severity-score"] = this_sin_score;
    result["evilness"] = result["severity-score"];
    result["timestamp_start"] = (Json::UInt64)flags_result.T_start;
    result["timestamp_stop"] = (Json::UInt64)flags_result.T_stop;
    root.append(result);

    add_severity_score(this_sin_score);
}


void Analyzer_Good_EKF::results_json_results(Json::Value &root)
{
    for (uint8_t i=0; i<next_result_variance;i++) {
        results_json_results_do_variance(root, &result_variance[i]);
    }
    for (uint8_t i=0; i<next_result_flags;i++) {
        results_json_results_do_flags(root, result_flags[i]);
    }

    std::map<const std::string, double> variances = _vehicle->ekf_variances();
    for (std::map<const std::string, double>::const_iterator it = variances.begin();
         it != variances.end();
         ++it) {
        std::string name = (*it).first;
        if (!_vehicle->ekf_variance_T(name)) {
            Json::Value result(Json::objectValue);
            result["timestamp"] = 0;
            result["status"] = "WARN";
            Json::Value reason(Json::arrayValue);
            reason.append(string_format("%s was never updated", name.c_str()));
            result["reason"] = reason;
            root.append(result);
        }
    }

    if (!_vehicle->ekf_flags_T()) {
        Json::Value result(Json::objectValue);
        result["timestamp"] = 0;
        result["status"] = "WARN";
        Json::Value reason(Json::arrayValue);
        reason.append("EKF flags were never updated");
        result["reason"] = reason;
        root.append(result);
    }

}
