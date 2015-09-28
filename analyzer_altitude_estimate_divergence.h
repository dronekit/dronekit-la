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

    void end_of_log(const uint32_t packet_count);

    void evaluate_estimate(
        std::string name,
        AnalyzerVehicle::Altitude altitude,
        AnalyzerVehicle::Altitude oestimate);
    void evaluate() override;

    void update_result_set_status(Analyzer_Estimate_Divergence_Result*) override;
    void open_result(std::string name, double delta);
    void close_result_add_evidence(Analyzer_Estimate_Divergence_Result *result) override;
    double maximum_altitude() { return _max_alt; }
    double maximum_altitude_relative() { return _max_alt_rel; }

private:

    double _max_alt = -1000;
    double _max_alt_rel = -1000;
    bool _was_armed = false;
    double _altitude_arm;
};

#endif
