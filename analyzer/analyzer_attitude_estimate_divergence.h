#ifndef ANALYZER_ATTITUDE_ESTIMATE_DIVERGENCE_H
#define ANALYZER_ATTITUDE_ESTIMATE_DIVERGENCE_H

/*
 * analyzer_attitude_estimate_divergence
 *
 */

#include "analyzer_estimate_divergence.h"

class Analyzer_Attitude_Estimate_Divergence_Result : public Analyzer_Estimate_Divergence_Result {
public:
    Analyzer_Attitude_Estimate_Divergence_Result(const std::string name) :
        Analyzer_Estimate_Divergence_Result(name)
        { }

private:
};

class Analyzer_Attitude_Estimate_Divergence : public Analyzer_Estimate_Divergence {

public:

    Analyzer_Attitude_Estimate_Divergence(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer_Estimate_Divergence(vehicle,data_sources)
    { }

    const std::string name() const override { return "Attitude Estimate Divergence"; }
    const std::string description() const override {
        return "This test will FAIL if various craft's attitude estimates diverge";
    }

    const std::string estimate_name() const {
        return "Attitude";
    };

    double default_delta_warn() const override { return 5.0f; }
    double default_delta_fail() const override { return 10.0f; }
    virtual uint64_t default_duration_min() const override { return 500000; }

    void evaluate_estimate(
        std::string name,
        AnalyzerVehicle::Attitude attitude,
        AnalyzerVehicle::Attitude estimate);
    void evaluate() override;

    void end_of_log(const uint32_t packet_count) override;

    const std::string _config_tag() const {
        return std::string("attitude_estimate_divergence");
    }

private:

    void open_result(std::string name, double delta);
    void update_result(std::string name, double delta);
    void close_result(std::string name);
    std::map<const std::string, Analyzer_Attitude_Estimate_Divergence_Result*> _result;
};

#endif
