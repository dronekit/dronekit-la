#ifndef ANALYZE_NOTCRASHED_H
#define ANALYZE_NOTCRASHED_H

/*
 * analyze_notcrashed
 *
 */

#include "analyzer.h"

class Analyzer_NotCrashed_Result : public Analyzer_Result_Event {
public:
    float angle;
    float angle_max;
    double servo_output[17]; // FIXME take 17 from somewhere...
};

// FIXME: name and scope:
class Analyzer_NotCrashed_Result_CRASHED : public Analyzer_Result_Event {
public:
};

class Analyzer_NotCrashed : public Analyzer {

public:

    Analyzer_NotCrashed(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    { }

    const uint16_t servo_output_threshold = 1250;
    
    const std::string name() const override { return "Crash Test"; }
    const std::string description() const override {
        return "Crashes are detected both heuristically and by explicit log messages.  This test will FAIL if the vehicle appears to crash";
    }

    void end_of_log(const uint32_t packet_count) override;

private:
    void evaluate_vehicle();
    void evaluate_attitude();

    void evaluate() override;

    Analyzer_NotCrashed_Result *_result = NULL;
    bool _was_crashed = false;
};

#endif
