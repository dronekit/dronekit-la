#ifndef ANALYZER_VEHICLE_DEFINITION_H
#define ANALYZER_VEHICLE_DEFINITION_H

/*
 * analyzer_vehicle_definition
 *
 */

#include "analyzer.h"

class Analyzer_Vehicle_Definition : public Analyzer {

public:
    Analyzer_Vehicle_Definition(AnalyzerVehicle::Base *&vehicle) :
	Analyzer(vehicle)
    { }

    const char *name() const override { return "Vehicle Defintion"; }
    const char *description() const override {
        return "This test will FAIL if the craft type is never defined";
    }

    void evaluate() override;

    void results_json_results(Json::Value &root);
private:
    bool vehicle_invalid = true;
};

#endif
