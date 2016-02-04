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

    const std::string estimate_name() const override {
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


    const std::string _config_tag() const override {
        return std::string("attitude_estimate_divergence");
    }

protected:

    const char *units() override { return "degrees"; }
    Analyzer_Attitude_Estimate_Divergence_Result* new_result_object(const std::string name) override;
    void open_result_add_data_sources(const std::string name) override;

    uint8_t severity_score_fail() override { return 10; }
    uint8_t severity_score_warn() override { return 10; }

private:

};

#endif
