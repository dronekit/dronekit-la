#include "analyzer_gyro_drift.h"

#include <stdio.h>
#include "util.h"
#include "analyzer_util.h"
#include <algorithm>

#include "analyzervehicle_copter.h"

void Analyzer_Gyro_Drift_Result::to_json(Json::Value &root) const
{
    Analyzer_Result_Period::to_json(root);

    root["axis"] = std::string("").append(1,'X' + _axis);
}


bool Analyzer_Gyro_Drift::configure(INIReader *config) {

    if (!Analyzer::configure(config)) {
        return false;
    }

    _delta_fail = config->GetReal("loganalyzer", "gyro_drift::delta_fail", 0.1f);
    _delta_warn = config->GetReal("loganalyzer", "gyro_drift::delta_warn", 0.09f);
    _gyr_avg_usec = config->GetReal("loganalyzer", "gyro_drift::avg_period", 2000000);
    _duration_min = config->GetReal("loganalyzer", "gyro_drift::duration_min", 500000);

    return true;
}

void Analyzer_Gyro_Drift::open_result(const std::string result_key,
                                      const AnalyzerVehicle::IMU *first,
                                      const AnalyzerVehicle::IMU *imu,
                                      const uint8_t axis,
                                      const double delta)
{

    _result[result_key] = new Analyzer_Gyro_Drift_Result(imu->name());

    auto result = _result[result_key];

    result->set_T_start(_vehicle->T());
    result->set_reason("Gyroscope readings differ from first gyroscope");
    result->_axis = axis;

    // both first IMU and this IMU
    result->add_source(_data_sources.get(string_format("IMU_%s_GYR", first->name().c_str())));
    result->add_source(_data_sources.get(string_format("IMU_%s_GYR", imu->name().c_str())));

    result->set_status(analyzer_status_warn);
    result->set_severity_score(20);

    update_result(result, delta);
}

void Analyzer_Gyro_Drift::update_result(Analyzer_Gyro_Drift_Result *result, const double new_value)
{
    if (new_value > result->_max_delta) {
        result->_max_delta = new_value;
        if (new_value > _delta_fail) {
            result->set_status(analyzer_status_fail);
            result->set_severity_score(40);
        }
    }
}

void Analyzer_Gyro_Drift::close_result(const std::string result_key,
                                      Analyzer_Gyro_Drift_Result *result)
{
    result->set_T_stop(_vehicle->T());

    if (result->duration() < _duration_min) {
        _result.erase(result_key);
        delete result;
        return;
    }

    result->add_evidence(string_format("threshold-duration=%f", _duration_min/1000000.0f));
    result->add_evidence("threshold-duration-units=seconds");
    result->add_evidence(string_format("max-delta=%f",
                                       rad_to_deg(result->_max_delta)));
    result->add_evidence("max-delta-units=degrees/second");

    if (result->status() == analyzer_status_fail) {
        result->add_evidence(string_format("threshold=%f",
                                           rad_to_deg(_delta_fail)));
    } else {
        result->add_evidence(string_format("threshold=%f",
                                           rad_to_deg(_delta_warn)));
    }
    result->add_evidence("threshold-units=degrees/second");

    add_result(result);
    _result.erase(result_key);
}

void Analyzer_Gyro_Drift::evaluate_gyro(const AnalyzerVehicle::IMU *first,
                                        const AnalyzerVehicle::IMU *imu)
{
    Vector3f first_avg;

    if (!first->gyr_avg(_vehicle->T(), _gyr_avg_usec, first_avg)) {
        return;
    }
    Vector3f gyr;
    if (!imu->gyr_avg(_vehicle->T(), _gyr_avg_usec, gyr)) {
        return;
    }

    const Vector3f deltas = first_avg - gyr;

    for (uint8_t i=0; i<3; i++) {
        const double delta = fabs(deltas[i]);
        const bool problem = (delta > _delta_warn);

        const std::string result_key = (imu->name() + "_").append(1,('X' + i));

        if (problem) {
            if (_result.count(result_key)) {
                // incident is underway
                update_result(_result[result_key], delta);
            } else {
                open_result(result_key, first, imu, i, delta);
            }
        } else {
            // close any open result:
            if (_result.count(result_key)) {
                close_result(result_key, _result[result_key]);
            }
        }
    }
}

void Analyzer_Gyro_Drift::evaluate()
{
    auto next = _vehicle->imus().begin();
    if (next == _vehicle->imus().end()) {
        // no Gyros so far
        return;
    }

    auto first_imu = next->second;

    next++;

    while (next != _vehicle->imus().end()) {
        evaluate_gyro(first_imu, next->second);
        next++;
    }
}

void Analyzer_Gyro_Drift::close_results()
{
    auto next = _result.begin();
    while (next != _result.end()) {
        auto current = next;
        next++;
        close_result((*current).first, (*current).second);
    }
}

void Analyzer_Gyro_Drift::end_of_log(uint32_t packet_count UNUSED)
{
    // close off any existing result:
    close_results();
}
