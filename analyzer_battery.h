#ifndef ANALYZER_BATTERY_H
#define ANALYZER_BATTERY_H

/*
 * analyzer_battery
 *
 */

#include "analyzer.h"

class Analyzer_Battery : public Analyzer {

public:
    Analyzer_Battery(AnalyzerVehicle::Base *&vehicle) :
	Analyzer(vehicle)
    {
    }

    void evaluate() override;

    const char *name() const override { return "Battery"; }
    const char *description() const override {
        return "This test will FAIL if the battery level remaining falls below a threshold level";
    }
    bool configure(INIReader *config);
    bool has_failed();

    void results_json_results(Json::Value &root);

private:
    double lowest_battery_remaining_seen = 999999999.0f;
    uint64_t lowest_battery_remaining_seen_T = 0;

    const double low_battery_threshold = 15;

    bool seen_failsafe_battery_event = false;
    uint64_t seen_failsafe_battery_event_T = 0;
};

#endif


