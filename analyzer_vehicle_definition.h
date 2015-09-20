#ifndef ANALYZER_VEHICLE_DEFINITION_H
#define ANALYZER_VEHICLE_DEFINITION_H

/*
 * analyzer_vehicle_definition
 *
 */

#include "analyzer.h"

class Analyzer_Vehicle_Definition_Result : public Analyzer_Result_Summary {
public:
    Analyzer_Vehicle_Definition_Result() :
        Analyzer_Result_Summary()
        { }
private:
};

class Analyzer_Vehicle_Definition : public Analyzer {

public:

    Analyzer_Vehicle_Definition(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    { }

    const std::string name() const override { return "Vehicle Defintion"; }
    const std::string description() const override {
        return "This test will FAIL if the craft type is never defined";
    }

    void evaluate() override;

    void end_of_log(const uint32_t packet_count) override;

private:
    bool vehicle_invalid = true;
};

#endif
