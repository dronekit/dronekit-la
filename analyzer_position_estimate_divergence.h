#ifndef ANALYZER_POSITION_ESTIMATE_DIVERGENCE_H
#define ANALYZER_POSITION_ESTIMATE_DIVERGENCE_H

/*
 * analyzer_position_estimate_divergence
 *
 */

#include "analyzer.h"

class Analyzer_Position_Estimate_Divergence_Result : public Analyzer_Result_Period {
public:
    Analyzer_Position_Estimate_Divergence_Result() :
        Analyzer_Result_Period()
        { }

    std::string name = "";
    double max_delta;
    double delta_threshold;

    void to_json(Json::Value &root) const override;
private:
};

class Analyzer_Position_Estimate_Divergence : public Analyzer {

public:

    Analyzer_Position_Estimate_Divergence(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    { }

    const char *name() const override { return "Position Estimate Divergence"; }
    const char *description() const override {
        return "This test will FAIL if various craft's position estimates diverge";
    }

    void evaluate_estimate(
        std::string name,
        AnalyzerVehicle::Position position,
        AnalyzerVehicle::Position oestimate);
    void evaluate() override;
        
private:

    const float position_delta_warn = 4.0f;
    const float position_delta_fail = 5.0f;

    std::map<const std::string, Analyzer_Position_Estimate_Divergence_Result> _current_states;
};

#endif
