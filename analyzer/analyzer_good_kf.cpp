#include "analyzer_good_kf.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Good_KF::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }

    for (std::map<const std::string, kf_variance>::const_iterator it = _variances.begin();
         it != _variances.end();
         ++it) {
        std::string name = (*it).first;

        _variances[name].threshold_warn = config->GetReal(
            "loganalyzer",
            string_format("%s::variance::%s::threshold_warn",
                          shortname_lc().c_str(),
                          name.c_str()),
            0.5f);
        _variances[name].threshold_fail = config->GetReal(
            "loganalyzer",
            string_format("%s::variance::%s::threshold_fail",
                          shortname_lc().c_str(),
                          name.c_str()),
            1.0f);
    }

    return true;
}

void Analyzer_Good_KF::end_of_log(uint32_t packet_count UNUSED)
{
    std::map<const std::string, double> my_variances = variances();

    for (std::map<const std::string, double>::const_iterator it = my_variances.begin();
         it != my_variances.end();
         ++it) {
        // ::fprintf(stderr, "would evaluate (%s)\n", (*it).first.c_str());
        std::string name = (*it).first;
        if (_results[name] != NULL) {
            close_variance_result(name);
        }
    }
    if (_result_flags != NULL) {
        close_result_flags();
    }

    for (std::map<const std::string, double>::const_iterator it = my_variances.begin();
         it != my_variances.end();
         ++it) {
        std::string name = (*it).first;
        if (!variance_T(name)) {
            Analyzer_Result_Summary *summary = new Analyzer_Result_Summary();
            summary->set_status(analyzer_status_warn);
            summary->set_reason(string_format("%s was never updated", name.c_str()));
            summary->add_source(_data_sources.get(
                                    string_format("%s_VARIANCES_%s",
                                                  shortname().c_str(),
                                                  name.c_str())));
            add_result(summary);
        }
    }

    if (!flags_T()) {
        Analyzer_Result_Summary *summary = new Analyzer_Result_Summary();
        summary->set_status(analyzer_status_warn);
        summary->set_reason(string_format("%s flags were never updated",
                                          shortname().c_str()));
        summary->add_source(_data_sources.get(string_format("%s_FLAGS",
                                                            shortname().c_str())));
        add_result(summary);
    }

}

bool Analyzer_Good_KF::flags_bad(uint16_t flags)
{
    for (uint16_t i=1; i<EKF_STATUS_FLAGS_ENUM_END; i<<=1) {
        if (!(flags & i)) {
            return true;
        }
    }
    return false;
}

void Analyzer_Good_KF::close_variance_result(const std::string name)
{
    Analyzer_Good_KF_Result_Variance *result = _results[name];
    kf_variance *variance = result->variance();

    result->set_T_stop(_vehicle->T());
    result->add_evidence(string_format("max-variance=%f", result->max()));
    if (result->max() > variance->threshold_fail) {
        result->set_status(analyzer_status_fail);
        result->set_reason(string_format("%s exceeds fail threshold", name.c_str()));
        result->add_evidence(string_format("threshold=%f", variance->threshold_fail));
        result->increase_severity_score(10);
    } else if (result->max() > variance->threshold_warn) {
        result->set_status(analyzer_status_warn);
        result->set_reason(string_format("%s exceeds warn threshold", name.c_str()));
        result->add_evidence(string_format("threshold=%f", variance->threshold_warn));
        result->increase_severity_score(4);
    } else {
        // should not happen
        ::fprintf(stderr, "Have a result with max less than threshold?!\n");
    }
    if (_vehicle->any_acc_clipping()) {
        result->add_evidence("Vehicle Accelerometers Clipping");
    }
    add_result(result);
    _results[name] = NULL;
}

void Analyzer_Good_KF::evaluate_variance(kf_variance &variance, double value)
{

    Analyzer_Good_KF_Result_Variance *result = _results[variance.name];

    if (result == NULL) {
        if (value > variance.threshold_warn) {
            // we have exceeeded a threshold
            result = new Analyzer_Good_KF_Result_Variance();
            result->add_source(_data_sources.get(string_format("%s_VARIANCES_%s", shortname().c_str(), variance.name)));
            result->set_variance(&variance);
            result->set_T_start(_vehicle->T());
            result->set_max(value);
            _results[variance.name] = result;
        }
    } else {
        // event is underway
        if (value > result->max()) {
            // a new record!
            result->set_max(value);
        } else if (value > variance.threshold_warn) {
            // we continue to exceed variances...
        } else {
            // our variances have come down sufficiently to close this instance
            close_variance_result(variance.name);
        }
    }
}

void Analyzer_Good_KF::evaluate_variances()
{
    std::map<const std::string, double> my_variances = variances();
    
    for (std::map<const std::string, double>::const_iterator it = my_variances.begin();
         it != my_variances.end();
         ++it) {
        std::string name = (*it).first;
        if (_variances.count(name) == 0) {
            // set a flag and warn?
            fprintf(stderr, "Unknown variance %s\n", name.c_str());
            continue;
        }
        kf_variance &variance = _variances[name];
        evaluate_variance(variance, (*it).second);
    }
}

void Analyzer_Good_KF::close_result_flags()
{
    _result_flags->set_T_stop(_vehicle->T());
    add_result(_result_flags);
    _result_flags = NULL;
}

void Analyzer_Good_KF::open_result_flags(uint16_t flags)
{
    _result_flags = new Analyzer_Good_KF_Result_Flags();
    _result_flags->set_T_start(_vehicle->T()); 
    _result_flags->set_status(analyzer_status_fail);
    _result_flags->set_flags(flags);
    _result_flags->add_source(_data_sources.get(string_format("%s_FLAGS", shortname().c_str())));
    _result_flags->set_reason(string_format("The %s status report indicates a problem with the EKF", shortname().c_str()));
    _result_flags->set_flags(this->flags());

    _result_flags->add_evidence(string_format("flags=%d", flags));
    // TODO: put the flags and the "bad" descrption into a structure
    _result_flags->increase_severity_score(10);
    if (!(flags & EKF_ATTITUDE)) {
        _result_flags->increase_severity_score(1);
        _result_flags->add_evidence("attitude estimate bad");
    }
    if (!(flags & EKF_VELOCITY_HORIZ)) {
        _result_flags->increase_severity_score(1);
        _result_flags->add_evidence("horizontal velocity estimate bad");
    }
    if (!(flags & EKF_VELOCITY_VERT)) {
        _result_flags->increase_severity_score(1);
        _result_flags->add_evidence("vertical velocity estimate bad");
    }
    if (!(flags & EKF_POS_HORIZ_REL)) {
        _result_flags->increase_severity_score(1);
        _result_flags->add_evidence("horizontal position (relative) estimate bad");
    }
    if (!(flags & EKF_POS_HORIZ_ABS)) {
        _result_flags->increase_severity_score(1);
        _result_flags->add_evidence("horizontal position (absolute) estimate bad");
    }
    if (!(flags & EKF_POS_VERT_ABS)) {
        _result_flags->increase_severity_score(1);
        _result_flags->add_evidence("vertical position (absolute) estimate bad");
    }
    if (!(flags & EKF_POS_VERT_AGL)) {
        _result_flags->increase_severity_score(1);
        _result_flags->add_evidence("vertical position (above ground) estimate bad");
    }
    if (flags & EKF_CONST_POS_MODE) {
        _result_flags->increase_severity_score(1);
        _result_flags->add_evidence("In constant position mode (no abs or rel position)");
    }
    if (!(flags & EKF_PRED_POS_HORIZ_REL)) {
        _result_flags->increase_severity_score(1);
        _result_flags->add_evidence("Predicted horizontal position (relative) bad");
    }
    if (!(flags & EKF_PRED_POS_HORIZ_ABS)) {
        _result_flags->increase_severity_score(1);
        _result_flags->add_evidence("Predicted horizontal position (absolute) bad");
    }

    if (_vehicle->any_acc_clipping()) {
        _result_flags->add_evidence("Vehicle Accelerometers Clipping");
    }
}


// FIXME: should be taking timestamps from ekf_flags_t rather than
// _vehicle->T()
void Analyzer_Good_KF::evaluate_flags()
{
    if (_result_flags == NULL) {
        if (flags_T()) {
            if (flags_bad(flags())) {
                // start a new incident
                open_result_flags(flags());
            }
        }
    } else {
        // ::fprintf(stderr, "flags: %d\n", _vehicle->ekf_flags());
        // incident is under way
        if (_result_flags->flags() != flags()) {
            // close off the old one
            close_result_flags();
            _result_flags = NULL;
            if (flags_bad(flags())) {
                // new flags are also bad!
                open_result_flags(flags());
            }
        }
    }
}


void Analyzer_Good_KF::evaluate()
{
    evaluate_variances();
    evaluate_flags();
}
