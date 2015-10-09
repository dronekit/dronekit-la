#ifndef ANALYZER_POSITION_ESTIMATE_DIVERGENCE_H
#define ANALYZER_POSITION_ESTIMATE_DIVERGENCE_H

/*
 * analyzer_position_estimate_divergence
 *
 */

#include "analyzer_estimate_divergence.h"

class Analyzer_Position_Estimate_Divergence_Result : public Analyzer_Estimate_Divergence_Result {
public:
    Analyzer_Position_Estimate_Divergence_Result(const std::string name) :
        Analyzer_Estimate_Divergence_Result(name)
        { }
private:
};

class Analyzer_Position_Estimate_Divergence : public Analyzer_Estimate_Divergence {

public:

    Analyzer_Position_Estimate_Divergence(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer_Estimate_Divergence(vehicle,data_sources)
    { }

    const std::string name() const override { return "Position Estimate Divergence"; }
    const std::string description() const override {
        return "This test will FAIL if various craft's position estimates diverge";
    }

    const std::string estimate_name() const {
        return "Position";
    };

    void evaluate_estimate(
        std::string name,
        AnalyzerVehicle::Position position,
        AnalyzerVehicle::Position oestimate);
    void evaluate() override;

    void open_result(std::string name, double delta);
    void update_result(std::string name, double delta);
    void close_result(std::string name);

    void end_of_log(const uint32_t packet_count) override;

    double total_distance_travelled() { return _total_distance_travelled; }

    const std::string _config_tag() {
        return std::string("position_estimate_divergence");
    }

    const double default_delta_warn() const override { return 4.0f; }
    const double default_delta_fail() const override { return 5.0f; }
    const uint64_t default_duration_min() const override { return 500000; }

private:

    std::map<const std::string, Analyzer_Position_Estimate_Divergence_Result*> _result = { };

    double _total_distance_travelled = 0;
    AnalyzerVehicle::Position prevpos = { };
};

#endif
