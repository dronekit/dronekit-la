#include "analyzer_altitude_estimate_divergence.h"

#include "analyzer_util.h"

Analyzer_Altitude_Estimate_Divergence_Result* Analyzer_Altitude_Estimate_Divergence::new_result_object(const std::string name)
{
    return new Analyzer_Altitude_Estimate_Divergence_Result(name);
}

void Analyzer_Altitude_Estimate_Divergence::end_of_log(const uint32_t packet_count)
{
    Analyzer_Estimate_Divergence::end_of_log(packet_count);

    AnalyzerVehicle::Altitude alt = _vehicle->alt();
    if (_vehicle->is_armed()) {
        double delta = alt.alt() - _altitude_arm;
        if (delta > _max_alt_rel) {
            _max_alt_rel = delta;
        }
    }
}

void Analyzer_Altitude_Estimate_Divergence::evaluate_estimate(
    std::string name,
    AnalyzerVehicle::Altitude altitude,
    AnalyzerVehicle::Altitude estimate)
{
    if (estimate.alt_modtime() == 0) {
        // No estimate for this  yet
        return;
    }

    double delta = estimate.alt() - altitude.alt();

    bool failing = (fabs(delta) > delta_warn());
    // ::fprintf(stderr, "%s: delta=%f (%s)\n", name.c_str(), delta, (failing ? "FAILING" : ""));

    Analyzer_Altitude_Estimate_Divergence_Result *result =
        (Analyzer_Altitude_Estimate_Divergence_Result*)result_for_name(name);
    
    if (result == NULL) {
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

void Analyzer_Altitude_Estimate_Divergence::open_result_add_data_sources(const std::string name)
{
    _result[name]->add_source(_data_sources.get("ALTITUDE"));
    _result[name]->add_source(_data_sources.get(std::string("ALTITUDE_ESTIMATE_") + name));
}

void Analyzer_Altitude_Estimate_Divergence::evaluate()
{
    const std::map<const std::string, AnalyzerVehicle::AltitudeEstimate*> &estimates =
        _vehicle->altitude_estimates();
    AnalyzerVehicle::Altitude alt = _vehicle->alt();
    // ::fprintf(stderr, "Altitude: %.20f/%.20f\n", pos.lat(), pos.lon());
    if (alt.alt_modtime() == 0) {
        // No craft altitude yet
        return;
    }

    if (_was_armed != _vehicle->is_armed()) {
        if (_vehicle->is_armed()) {
            // moved from disarmed -> armed
            _altitude_arm = alt.alt();
        } else {
            // moved from armed -> disarmed
        }
    } else if (_vehicle->is_armed()) {
        double delta = alt.alt() - _altitude_arm;
        if (delta > _max_alt_rel) {
            _max_alt_rel = delta;
        }
    }
    _was_armed = _vehicle->is_armed();

    if (alt.alt() > _max_alt) {
        _max_alt = alt.alt();
    }

    for (std::map<const std::string, AnalyzerVehicle::AltitudeEstimate*>::const_iterator it = estimates.begin();
         it != estimates.end();
         ++it) {
        AnalyzerVehicle::AltitudeEstimate *est = (*it).second;
        AnalyzerVehicle::Altitude estimate = est->altitude();
        evaluate_estimate((*it).first, alt, estimate);
    }
}
