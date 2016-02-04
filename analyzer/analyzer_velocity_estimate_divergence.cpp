#include "analyzer_velocity_estimate_divergence.h"

#include "analyzer_util.h"

void Analyzer_Velocity_Estimate_Divergence::evaluate_estimate(
    const std::string name,
    AnalyzerVehicle::Velocity &velocity,
    AnalyzerVehicle::Velocity &estimate)
{
    if (estimate.velocity_modtime() == 0) {
        // No estimate for this  yet
        return;
    }

    // make sure we're comparing apples with apples:
    double velocity_size;
    if (estimate.is_2d()) {
        velocity_size = velocity.size_2d();
    } else {
        velocity_size = velocity.size();
    }
    double delta = estimate.size() - velocity_size;
    // ::fprintf(stderr, "delta: %f\n", delta);

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

Analyzer_Velocity_Estimate_Divergence_Result* Analyzer_Velocity_Estimate_Divergence::new_result_object(const std::string name)
{
    return new Analyzer_Velocity_Estimate_Divergence_Result(name);
}

void Analyzer_Velocity_Estimate_Divergence::open_result_add_data_sources(const std::string name)
{
    _result[name]->add_source(_data_sources.get("VELOCITY"));
    _result[name]->add_source(_data_sources.get(std::string("VELOCITY_ESTIMATE_") + name));
}

void Analyzer_Velocity_Estimate_Divergence::evaluate()
{
    const std::map<const std::string, AnalyzerVehicle::VelocityEstimate*> &estimates =
        _vehicle->velocity_estimates();
    AnalyzerVehicle::Velocity &vel = _vehicle->vel();
    // ::fprintf(stderr, "Velocity: %.20f", vel);
    if (vel.velocity_modtime() == 0) {
        // No craft velocity yet
        return;
    }
    for (std::map<const std::string, AnalyzerVehicle::VelocityEstimate*>::const_iterator it = estimates.begin();
         it != estimates.end();
         ++it) {
        AnalyzerVehicle::VelocityEstimate *est = (*it).second;
        AnalyzerVehicle::Velocity &estimate = est->velocity();
        evaluate_estimate((*it).first, vel, estimate);
    }

    if (vel.size() > _maximum_velocity) {
        _maximum_velocity = vel.size();
    }
}
