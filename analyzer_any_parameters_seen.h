#ifndef ANALYZER_ANY_PARAMETERS_SEEN_H
#define ANALYZER_ANY_PARAMETERS_SEEN_H

/*
 * analyzer_any_parameters_seen
 *
 */

#include "analyzer.h"

class Analyzer_Any_Parameters_Seen : public Analyzer {

public:
    Analyzer_Any_Parameters_Seen(AnalyzerVehicle::Base *&vehicle) :
	Analyzer(vehicle)
    { }

    const char *name() const override { return "Any Parameters Seen"; }
    const char *description() const override {
        return "This test will FAIL if no parameters are seen from the UAV";
    }

    void evaluate() override;

    void results_json_results(Json::Value &root) override;
private:
    bool any_parameters_seen = false;
};

#endif
