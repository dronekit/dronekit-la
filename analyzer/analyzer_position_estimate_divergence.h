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

    virtual ~Analyzer_Position_Estimate_Divergence() { }

    const std::string estimate_name() const override {
        return "Position";
    };

    void evaluate_estimate(
        std::string name,
        AnalyzerVehicle::Position position,
        AnalyzerVehicle::Position oestimate);
    void evaluate() override;

    double total_distance_travelled() { return _total_distance_travelled; }
    double maximum_distance_from_origin() { return _maximum_distance_from_origin; }

    const std::string _config_tag() const override {
        return std::string("position_estimate_divergence");
    }

    double default_delta_warn() const override { return 4.0f; }
    double default_delta_fail() const override { return 5.0f; }
    uint64_t default_duration_min() const override { return 500000; }

protected:

    Analyzer_Position_Estimate_Divergence_Result* new_result_object(const std::string name) override;
    virtual void open_result_add_data_sources(const std::string name) override;
    const char *units() override { return "metres"; }

private:

    double _total_distance_travelled = 0;
    AnalyzerVehicle::Position prevpos = { };

    double _maximum_distance_from_origin = 0;
};

#endif
