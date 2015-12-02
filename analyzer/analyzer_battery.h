#ifndef ANALYZER_BATTERY_H
#define ANALYZER_BATTERY_H

/*
 * analyzer_battery
 *
 */

#include "analyzer.h"

class Analyzer_Battery_Result : public Analyzer_Result_Summary {
public:
    Analyzer_Battery_Result() :
        Analyzer_Result_Summary()
        { }
};


class Analyzer_Battery : public Analyzer {

public:

    Analyzer_Battery(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    {
        result = new Analyzer_Battery_Result();
        result->add_source(_data_sources.get("BATTERY_REMAINING"));
    }

    void evaluate() override;

    const std::string name() const override { return "Battery"; }
    const std::string description() const override {
        return "This test will FAIL if the battery level falls below the battery failsafe threshold level, or if a battery failsafe event is received.";
    }
    bool configure(INIReader *config) override;

    void end_of_log(const uint32_t packet_count) override;

private:
    Analyzer_Battery_Result *result;
    double lowest_battery_remaining_seen = 999999999.0f;
    uint64_t lowest_battery_remaining_seen_T = 0;

    double low_battery_threshold = 15.0f;

    bool seen_failsafe_battery_event = false;
    uint64_t seen_failsafe_battery_event_T = 0;
};

#endif


