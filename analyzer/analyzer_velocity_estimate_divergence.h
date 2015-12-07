#ifndef ANALYZER_VELOCITY_ESTIMATE_DIVERGENCE_H
#define ANALYZER_VELOCITY_ESTIMATE_DIVERGENCE_H

/*
 * analyzer_velocity_estimate_divergence
 *
 */

#include "analyzer_estimate_divergence.h"

// class Analyzer_Velocity_Estimate_Divergence_Result : public Analyzer_Estimate_Divergence_Result {
// public:
//     Analyzer_Velocity_Estimate_Divergence_Result(const std::string name) :
//         Analyzer_Estimate_Divergence_Result(name)
//         { }
// private:
// };

class Analyzer_Velocity_Estimate_Divergence : public Analyzer_Estimate_Divergence {

public:

    Analyzer_Velocity_Estimate_Divergence(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer_Estimate_Divergence(vehicle,data_sources)
    { }

    const std::string name() const override { return "Velocity Estimate Divergence"; }
    const std::string description() const override {
        return "A UAV typically has several estimates of its velocity.  This test will FAIL if the craft's velocity estimates diverge.";
    }

    const std::string estimate_name() const {
        return "Velocity";
    };

    void evaluate_estimate(
        std::string name,
        AnalyzerVehicle::Velocity velocity,
        AnalyzerVehicle::Velocity oestimate);
    void evaluate() override;

    // void open_result(std::string name, double delta);
    // void update_result(std::string name, double delta);
    // void close_result(std::string name);

    void end_of_log(const uint32_t packet_count) override;

    double maximum_velocity() { return _maximum_velocity; }

    const std::string _config_tag() const {
        return std::string("velocity_estimate_divergence");
    }

    // double default_delta_warn() const override { return 4.0f; }
    // double default_delta_fail() const override { return 5.0f; }
    // uint64_t default_duration_min() const override { return 500000; }

private:

    // std::map<const std::string, Analyzer_Velocity_Estimate_Divergence_Result*> _result = { };

    double _maximum_velocity = 0;
};

#endif
