#ifndef ANALYZER_VELOCITY_ESTIMATE_DIVERGENCE_H
#define ANALYZER_VELOCITY_ESTIMATE_DIVERGENCE_H

/*
 * analyzer_velocity_estimate_divergence
 *
 */

#include "analyzer_estimate_divergence.h"

class Analyzer_Velocity_Estimate_Divergence_Result : public Analyzer_Estimate_Divergence_Result {
public:
    Analyzer_Velocity_Estimate_Divergence_Result(const std::string name) :
        Analyzer_Estimate_Divergence_Result(name)
        { }
private:
};

class Analyzer_Velocity_Estimate_Divergence : public Analyzer_Estimate_Divergence {

public:

    Analyzer_Velocity_Estimate_Divergence(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer_Estimate_Divergence(vehicle,data_sources)
    { }

    virtual ~Analyzer_Velocity_Estimate_Divergence() { }

    const std::string estimate_name() const override {
        return "Velocity";
    };

    void evaluate_estimate(
        std::string name,
        AnalyzerVehicle::Velocity &velocity,
        AnalyzerVehicle::Velocity &estimate);
    void evaluate() override;

    double maximum_velocity() { return _maximum_velocity; }

    const std::string _config_tag() const override {
        return std::string("velocity_estimate_divergence");
    }

    double default_delta_warn() const override { return 4.0f; }
    double default_delta_fail() const override { return 5.0f; }
    uint64_t default_duration_min() const override { return 500000; }

protected:

    Analyzer_Velocity_Estimate_Divergence_Result* new_result_object(const std::string name) override;
    void open_result_add_data_sources(const std::string name) override;

private:

    const char *units() override { return "metres/second"; }

    double _maximum_velocity = 0;
};

#endif
