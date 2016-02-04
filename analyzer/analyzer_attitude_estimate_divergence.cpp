#include "analyzer_attitude_estimate_divergence.h"

#include "analyzer_util.h"

Analyzer_Attitude_Estimate_Divergence_Result* Analyzer_Attitude_Estimate_Divergence::new_result_object(const std::string name)
{
    return new Analyzer_Attitude_Estimate_Divergence_Result(name);
}

void Analyzer_Attitude_Estimate_Divergence::open_result_add_data_sources(const std::string name)
{
    _result[name]->add_source(_data_sources.get("ATTITUDE"));
    _result[name]->add_source(_data_sources.get(std::string("ATTITUDE_ESTIMATE_") + name));
}

double angle_delta(double a, double b)
{
    double delta = b - a;
    if (delta > 180) {
        delta = delta - 360;
    } else if (delta < -180) {
        delta += 360;
    }
    // ::fprintf(stderr, "delta (%f, %f) = %f\n", a, b, delta);
    return delta;
}
void Analyzer_Attitude_Estimate_Divergence::evaluate_estimate(
    std::string name,
    AnalyzerVehicle::Attitude attitude,
    AnalyzerVehicle::Attitude estimate)
{
    if (estimate.roll_modtime() == 0) {
        // No estimate for this  yet
        return;
    }

    double delta_roll = fabs(angle_delta(estimate.roll(), attitude.roll()));
    double delta_pitch = fabs(angle_delta(estimate.pitch(), attitude.pitch()));
    // double delta_yaw = estimate.yaw() - attitude.yaw();

    double delta = delta_roll > delta_pitch ? delta_roll : delta_pitch;

    bool failing = (delta >= delta_warn());
    if (_result[name] == NULL) {
        if (failing) {
            open_result(name, delta);
        }
    } else {
        if (failing) {
            update_result(name, delta);
        } else {
            close_result(name);
        }
    }
}

void Analyzer_Attitude_Estimate_Divergence::evaluate()
{
    const std::map<const std::string, AnalyzerVehicle::AttitudeEstimate*> &estimates =
        _vehicle->attitude_estimates();
    AnalyzerVehicle::Attitude att = _vehicle->att();
    // ::fprintf(stderr, "Attitude: %.20f/%.20f\n", att.lat(), att.lon());
    if (att.roll_modtime() == 0) {
        // No craft attitude yet
        return;
    }
    for (std::map<const std::string, AnalyzerVehicle::AttitudeEstimate*>::const_iterator it = estimates.begin();
         it != estimates.end();
         ++it) {
        AnalyzerVehicle::AttitudeEstimate *est = (*it).second;
        AnalyzerVehicle::Attitude estimate = est->attitude();
        evaluate_estimate((*it).first, att, estimate);
    }
}
