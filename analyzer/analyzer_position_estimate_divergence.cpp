#include "analyzer_position_estimate_divergence.h"

#include "analyzer_util.h"

void Analyzer_Position_Estimate_Divergence::evaluate_estimate(
    const std::string name,
    AnalyzerVehicle::Position position,
    AnalyzerVehicle::Position estimate)
{
    if (estimate.lat_modtime() == 0) {
        // No estimate for this  yet
        return;
    }
    double delta = estimate.horizontal_distance_to(position);

    bool failing = (delta > delta_warn());
    // ::fprintf(stderr, "%s: delta=%f delta_warn=%f: %s\n", name.c_str(), delta, delta_warn(), failing ? "FAILING" : "");

    if (_result.count(name) == 0 ||
        _result[name] == NULL) {
        // no current problem
        if (failing) {
            open_result(name, delta);
        }
    } else {
        // problem currently underway
        if (failing) {
            update_result(name, delta);
        } else {
            close_result(name);
        }
    }
}

Analyzer_Position_Estimate_Divergence_Result* Analyzer_Position_Estimate_Divergence::new_result_object(const std::string name)
{
    return new Analyzer_Position_Estimate_Divergence_Result(name);
}

void Analyzer_Position_Estimate_Divergence::open_result_add_data_sources(const std::string name)
{
    _result[name]->add_source(_data_sources.get("POSITION"));
    _result[name]->add_source(_data_sources.get(std::string("POSITION_ESTIMATE_") + name));
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

    if (!prevpos.is_zero_zero() &&
        !pos.is_zero_zero()) {
        _total_distance_travelled += prevpos.horizontal_distance_to(pos);
        const double dfo = _vehicle->distance_from_origin();
        if (!is_equal(dfo, -1)) {
            if (dfo > _maximum_distance_from_origin) {
                _maximum_distance_from_origin = dfo;
            }
        }
    }
    prevpos = pos;
}
