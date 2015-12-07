#ifndef ANALYZER_ANY_PARAMETERS_SEEN_H
#define ANALYZER_ANY_PARAMETERS_SEEN_H

/*
 * analyzer_any_parameters_seen
 *
 */

#include "analyzer.h"

class Analyzer_Any_Parameters_Seen_Result : public Analyzer_Result_Summary {
public:
    Analyzer_Any_Parameters_Seen_Result() :
        Analyzer_Result_Summary()
        { }
private:
};

class Analyzer_Any_Parameters_Seen : public Analyzer {

public:

    Analyzer_Any_Parameters_Seen(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer(vehicle,data_sources)
    { }

    const std::string name() const override { return "Any Parameters Seen"; }
    const std::string description() const override {
        return "Autopilots store configuration settings known as 'parameters'. For proper analysis, logs must contain this parameter information. This test will FAIL if the input does not contain parameter information.";
    }

    void evaluate() override;

    void end_of_log(const uint32_t packet_count) override;

private:
    bool any_parameters_seen = false;
};

#endif
