#include "analyzer_position_estimate_divergence.h"

#include "analyzer_util.h"

void Analyzer_Position_Estimate_Divergence_Result::to_json(Json::Value &root) const
{
    Analyzer_Result_Period::to_json(root);
    root["name"] = name;
}

void Analyzer_Position_Estimate_Divergence::evaluate_estimate(
    std::string name,
    AnalyzerVehicle::Position position,
    AnalyzerVehicle::Position estimate)
{
    if (estimate.lat_modtime() == 0) {
        // No estimate for this  yet
        return;
    }
    double delta = estimate.horizontal_distance_to(position);

    Analyzer_Position_Estimate_Divergence_Result &result =
        _current_states[name];
    if (result.name == "") {
        result.name = name;
    }
    analyzer_status new_status;
    // ::fprintf(stderr, "%s=(%f/%f) canon=(%f/%f) %f\n", name.c_str(), estimate.lat(), estimate.lon(),position.lat(), position.lon(), delta);
    if (delta > position_delta_fail) { // FIXME magic number
        new_status = analyzer_status_fail;
    } else if (delta > position_delta_warn && result.status() != analyzer_status_fail) { // FIXME magic number
        new_status = analyzer_status_warn;
    } else {
        new_status = analyzer_status_ok;
    }

    if (new_status == result.status()) {
        // same status as before; see if we can update i
        if (result.status() != analyzer_status_ok) {
            if (delta > result.max_delta) {
                result.max_delta = delta;
            }
        }
    } else {
        // status has changed
        if (result.status() != analyzer_status_ok) {
            if (new_status == analyzer_status_ok) {
                // check minimum duration:
                if (_vehicle->T() - result._T_start > 500000) {
                    // not moving from OK and not moving from fail back to warn
                    // lock this result in and move onto another
                    result._T_stop = _vehicle->T();
                    Analyzer_Position_Estimate_Divergence_Result *copy = new Analyzer_Position_Estimate_Divergence_Result();
                    copy->set_reason("This position estimate differs from the canonical craft position");
                    copy->max_delta = result.max_delta;
                    copy->delta_threshold = result.delta_threshold;
                    copy->_T_start = result._T_start;
                    copy->_T_stop = result._T_stop;
                    copy->set_status(result.status());
                    copy->name = result.name;
                    copy->add_evidence(string_format("max-delta=%f metres", result.max_delta));
                    copy->add_evidence(string_format("delta-threshold=%f metres", result.delta_threshold));
                    copy->set_evilness(result.evilness());
                    copy->add_series(_data_sources.get("POSITION"));
                    copy->add_series(_data_sources.get(std::string("POSITION_ESTIMATE_") + name));
                    add_result(copy);
                }
            }
        }
        result.max_delta = delta;
        result._T_start = _vehicle->T();
        result.set_status(new_status);
        result.delta_threshold = (new_status == analyzer_status_warn ? position_delta_warn : position_delta_fail); // FIXME
        switch(new_status) {
        case analyzer_status_warn:
            result.set_evilness(10);
            break;
        case analyzer_status_fail:
            result.set_evilness(20);
            break;
        case analyzer_status_ok:
            break;
        }
    }

        
}
void Analyzer_Position_Estimate_Divergence::evaluate()
{
    const std::map<const std::string, AnalyzerVehicle::PositionEstimate*> &estimates =
        _vehicle->position_estimates();
    AnalyzerVehicle::Position pos = _vehicle->pos();
    // ::fprintf(stderr, "Position: %.20f/%.20f\n", pos.lat(), pos.lon());
    if (pos.lat_modtime() == 0) {
        // No craft position yet
        return;
    }
    for (std::map<const std::string, AnalyzerVehicle::PositionEstimate*>::const_iterator it = estimates.begin();
         it != estimates.end();
         ++it) {
        AnalyzerVehicle::PositionEstimate *est = (*it).second;
        AnalyzerVehicle::Position estimate = est->position();
        evaluate_estimate((*it).first, pos, estimate);
    }
}
