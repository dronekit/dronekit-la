#include "analyzer_good_ekf.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Good_EKF::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Good_EKF::end_of_log(uint32_t packet_count)
{
    std::map<const std::string, double> variances = _vehicle->ekf_variances();
    
    for (std::map<const std::string, double>::const_iterator it = variances.begin();
         it != variances.end();
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

    for (std::map<const std::string, double>::const_iterator it = variances.begin();
         it != variances.end();
         ++it) {
        std::string name = (*it).first;
        if (!_vehicle->ekf_variance_T(name)) {
            Analyzer_Result_Summary *summary = new Analyzer_Result_Summary();
            summary->set_status(analyzer_status_warn);
            summary->set_reason(string_format("%s was never updated", name.c_str()));
            summary->add_source(_data_sources.get(string_format("EKF_VARIANCES_%s", name.c_str())));
            add_result(summary);
        }
    }

    if (!_vehicle->ekf_flags_T()) {
        Analyzer_Result_Summary *summary = new Analyzer_Result_Summary();
        summary->set_status(analyzer_status_warn);
        summary->set_reason("EKF flags were never updated");
        summary->add_source(_data_sources.get("EKF_FLAGS"));
        add_result(summary);
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

void Analyzer_Good_EKF::close_variance_result(const std::string name)
{
    Analyzer_Good_EKF_Result_Variance *result = _results[name];
    ekf_variance *variance = result->variance();

    result->set_T_stop(_vehicle->T());
    result->add_evidence(string_format("max-variance=%f", result->max()));
    if (result->max() > variance->threshold_fail) {
        result->set_status(analyzer_status_fail);
        result->set_reason(string_format("%s exceeds fail threshold", name.c_str()));
        result->add_evidence(string_format("threshold=%f", variance->threshold_fail));
        result->add_evilness(10);
    } else if (result->max() > variance->threshold_warn) {
        result->set_status(analyzer_status_warn);
        result->set_reason(string_format("%s exceeds warn threshold", name.c_str()));
        result->add_evidence(string_format("threshold=%f", variance->threshold_warn));
        result->add_evilness(4);
    } else {
        // should not happen
        ::fprintf(stderr, "Have a result with max less than threshold?!\n");
    }
    add_result(result);
    _results[name] = NULL;
}

void Analyzer_Good_EKF::evaluate_variance(ekf_variance &variance, double value)
{

    Analyzer_Good_EKF_Result_Variance *result = _results[variance.name];

    if (result == NULL) {
        if (value > variance.threshold_warn) {
            // we have exceeeded a threshold
            result = new Analyzer_Good_EKF_Result_Variance();
            result->add_source(_data_sources.get(string_format("EKF_VARIANCES_%s", variance.name)));
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

void Analyzer_Good_EKF::evaluate_variances()
{
    std::map<const std::string, double> variances = _vehicle->ekf_variances();
    
    for (std::map<const std::string, double>::const_iterator it = variances.begin();
         it != variances.end();
         ++it) {
        std::string name = (*it).first;
        if (_variances.count(name) == 0) {
            // set a flag and warn?
            fprintf(stderr, "Unknown variance %s\n", name.c_str());
            continue;
        }
        ekf_variance &variance = _variances[name];
        evaluate_variance(variance, (*it).second);
    }
}

void Analyzer_Good_EKF::close_result_flags()
{
    _result_flags->set_T_stop(_vehicle->T());
    add_result(_result_flags);
    _result_flags = NULL;
}

void Analyzer_Good_EKF::open_result_flags(uint16_t flags)
{
    _result_flags = new Analyzer_Good_EKF_Result_Flags();
    _result_flags->set_T_start(_vehicle->T()); 
    _result_flags->set_status(analyzer_status_fail);
    _result_flags->set_flags(flags);
    _result_flags->add_source(_data_sources.get("EKF_FLAGS"));
    _result_flags->set_reason("The EKF status report indicates a problem with the EKF");
    _result_flags->set_flags(_vehicle->ekf_flags());

    _result_flags->add_evidence(string_format("flags=%d", flags));
    // TODO: put the flags and the "bad" descrption into a structure
    _result_flags->add_evilness(10);
    if (!(flags & EKF_ATTITUDE)) {
        _result_flags->add_evilness(1);
        _result_flags->add_evidence("attitude estimate bad");
    }
    if (!(flags & EKF_VELOCITY_HORIZ)) {
        _result_flags->add_evilness(1);
        _result_flags->add_evidence("horizontal velocity estimate bad");
    }
    if (!(flags & EKF_VELOCITY_VERT)) {
        _result_flags->add_evilness(1);
        _result_flags->add_evidence("vertical velocity estimate bad");
    }
    if (!(flags & EKF_POS_HORIZ_REL)) {
        _result_flags->add_evilness(1);
        _result_flags->add_evidence("horizontal position (relative) estimate bad");
    }
    if (!(flags & EKF_POS_HORIZ_ABS)) {
        _result_flags->add_evilness(1);
        _result_flags->add_evidence("horizontal position (absolute) estimate bad");
    }
    if (!(flags & EKF_POS_VERT_ABS)) {
        _result_flags->add_evilness(1);
        _result_flags->add_evidence("vertical position (absolute) estimate bad");
    }
    if (!(flags & EKF_POS_VERT_AGL)) {
        _result_flags->add_evilness(1);
        _result_flags->add_evidence("vertical position (above ground) estimate bad");
    }
    if (!(flags & EKF_CONST_POS_MODE)) {
        _result_flags->add_evilness(1);
        _result_flags->add_evidence("In constant position mode (no abs or rel position)");
    }
    if (!(flags & EKF_PRED_POS_HORIZ_REL)) {
        _result_flags->add_evilness(1);
        _result_flags->add_evidence("Predicted horizontal position (relative) bad");
    }
    if (!(flags & EKF_PRED_POS_HORIZ_ABS)) {
        _result_flags->add_evilness(1);
        _result_flags->add_evidence("Predicted horizontal position (absolute) bad");
    }
}


// FIXME: should be taking timestamps from ekf_flags_t rather than
// _vehicle->T()
void Analyzer_Good_EKF::evaluate_flags()
{
    if (_result_flags == NULL) {
        if (ekf_flags_bad(_vehicle->ekf_flags())) {
            // start a new incident
            open_result_flags(_vehicle->ekf_flags());
        }
    } else {
        // ::fprintf(stderr, "flags: %d\n", _vehicle->ekf_flags());
        // incident is under way
        if (_result_flags->flags() != _vehicle->ekf_flags()) {
            // close off the old one
            close_result_flags();
            _result_flags = NULL;
            if (ekf_flags_bad(_vehicle->ekf_flags())) {
                // new flags are also bad!
                open_result_flags(_vehicle->ekf_flags());
            }
        }
    }
}


void Analyzer_Good_EKF::evaluate()
{
    evaluate_variances();
    evaluate_flags();
}
