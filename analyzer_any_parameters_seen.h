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

    const char *name() const override { return "Any Parameters Seen"; }
    const char *description() const override {
        return "This test will FAIL if no parameters are seen from the UAV";
    }

    void evaluate() override;

    void end_of_log(const uint32_t packet_count) override;

private:
    bool any_parameters_seen = false;
};

#endif
