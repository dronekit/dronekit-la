#ifndef ANALYZER_ALTITUDE_ESTIMATE_DIVERGENCE_H
#define ANALYZER_ALTITUDE_ESTIMATE_DIVERGENCE_H

/*
 * analyzer_altitude_estimate_divergence
 *
 */

#include "analyzer_estimate_divergence.h"

class Analyzer_Altitude_Estimate_Divergence_Result : public Analyzer_Estimate_Divergence_Result {
public:
    Analyzer_Altitude_Estimate_Divergence_Result(std::string name) :
        Analyzer_Estimate_Divergence_Result(name)
        { }
private:
};

class Analyzer_Altitude_Estimate_Divergence : public Analyzer_Estimate_Divergence {

public:

    Analyzer_Altitude_Estimate_Divergence(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer_Estimate_Divergence(vehicle,data_sources)
    { }

    const std::string estimate_name() const {
        return "Altitude";
    };

    void evaluate_estimate(
        std::string name,
        AnalyzerVehicle::Altitude altitude,
        AnalyzerVehicle::Altitude oestimate);
    void evaluate() override;

    void update_result_set_status(Analyzer_Estimate_Divergence_Result*) override;
    void open_result(std::string name, double delta);
    void close_result_add_evidence(Analyzer_Estimate_Divergence_Result *result) override;

private:
};

#endif
