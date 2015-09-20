#include "analyzer_altitude_estimate_divergence.h"

#include "analyzer_util.h"

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

    bool failing = (delta > delta_warn());

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

void Analyzer_Altitude_Estimate_Divergence::update_result_set_status(Analyzer_Estimate_Divergence_Result *result)
{
    // if (multiple_values_seen) {
    //     Analyzer_Estimate_Divergence::update_result_set_status(result);
    // } else {
        result->set_status(analyzer_status_warn);
    // }
}

void Analyzer_Altitude_Estimate_Divergence::open_result(const std::string name,
                                                        double delta)
{
    Analyzer_Altitude_Estimate_Divergence_Result *result =
        new Analyzer_Altitude_Estimate_Divergence_Result(name);
    result->set_reason("This altitude estimate differs from the canonical craft altitude");
    result->set_T_start(_vehicle->T());
    result->set_max_delta(0);
    result->add_series(_data_sources.get("ALTITUDE"));
    result->add_series(_data_sources.get(std::string("ALTITUDE_ESTIMATE_") + name));
    set_result_for_name(name, result);
    update_result(name, delta);
}

void Analyzer_Altitude_Estimate_Divergence::close_result_add_evidence(Analyzer_Estimate_Divergence_Result *result) {
    Analyzer_Estimate_Divergence::close_result_add_evidence(result);
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
    for (std::map<const std::string, AnalyzerVehicle::AltitudeEstimate*>::const_iterator it = estimates.begin();
         it != estimates.end();
         ++it) {
        AnalyzerVehicle::AltitudeEstimate *est = (*it).second;
        AnalyzerVehicle::Altitude estimate = est->altitude();
        evaluate_estimate((*it).first, alt, estimate);
    }
}
