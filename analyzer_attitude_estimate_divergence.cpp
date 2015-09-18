#include "analyzer_attitude_estimate_divergence.h"

#include "analyzer_util.h"

void Analyzer_Attitude_Estimate_Divergence_Result::to_json(Json::Value &root) const
{
    Analyzer_Result_Period::to_json(root);
    root["name"] = _name;
}

void Analyzer_Attitude_Estimate_Divergence::end_of_log(const uint32_t packet_count)
{
    for (std::map<const std::string, Analyzer_Attitude_Estimate_Divergence_Result*>::iterator it = _result.begin();
         it != _result.end();
         ++it) {
        Analyzer_Attitude_Estimate_Divergence_Result *result = (*it).second;
        if (result != NULL) {
            close_result((*it).first);
        }        
    }
}

void Analyzer_Attitude_Estimate_Divergence::open_result(std::string name,
                                                        double delta)
{
    _result[name] = new Analyzer_Attitude_Estimate_Divergence_Result();
    _result[name]->set_name(name);
    _result[name]->set_reason("This attitude estimate differs from the canonical craft attitude");
    _result[name]->set_T_start(_vehicle->T());
    _result[name]->set_max_delta(0);
    _result[name]->add_series(_data_sources.get("ATTITUDE"));
    _result[name]->add_series(_data_sources.get(std::string("ATTITUDE_ESTIMATE_") + name));
    update_result(name, delta);
}

void Analyzer_Attitude_Estimate_Divergence::update_result(std::string name,
                                                          double delta)
{
    if (delta > _result[name]->max_delta()) {
        _result[name]->set_max_delta(delta);
        if (delta >= attitude_max_delta_roll_pitch_fail) {
            _result[name]->set_delta_threshold(attitude_max_delta_roll_pitch_fail);
            _result[name]->set_status(analyzer_status_fail);
        } else if (delta >= attitude_max_delta_roll_pitch_warn) {
            _result[name]->set_status(analyzer_status_warn);
            _result[name]->set_delta_threshold(attitude_max_delta_roll_pitch_warn);
        }
    }
}

void Analyzer_Attitude_Estimate_Divergence::close_result(std::string name)
{
    _result[name]->set_T_stop(_vehicle->T());
    if (_result[name]->duration() < delta_time_threshold) {
        // event was too short; ignore it
        delete _result[name];
        _result[name] = NULL;
        return;
    }
    
    _result[name]->add_evidence(string_format("max-delta=%f degrees", _result[name]->max_delta()));
    _result[name]->add_evidence(string_format("delta-threshold=%f degrees", _result[name]->delta_threshold()));
    _result[name]->add_evidence(string_format("delta-time-threshold=%f seconds", delta_time_threshold / 1000000.0f));
    _result[name]->set_evilness(10);

    add_result(_result[name]);
    _result[name] = NULL;
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

    bool failing = (delta >= attitude_max_delta_roll_pitch_warn);
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
