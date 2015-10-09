#ifndef ANALYZE_NOTCRASHED_H
#define ANALYZE_NOTCRASHED_H

/*
 * analyze_notcrashed
 *
 */

#include "analyzer.h"

class Analyzer_NotCrashed_Result : public Analyzer_Result_Event {
public:
    uint64_t timestamp_start;
    uint64_t timestamp_stop;
    float angle;
    float angle_max;
    double servo_output[17]; // FIXME take 17 from somewhere...
    uint64_t duration() { return (timestamp_stop - timestamp_start); }
};

class Analyzer_NotCrashed : public Analyzer {

public:

    Analyzer_NotCrashed(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    { }

    const uint16_t servo_output_threshold = 1250;
    
    const std::string name() const override { return "Crash Test"; }
    const std::string description() const override {
        return "This test will FAIL if the vehicle appears to crash";
    }

    void end_of_log(const uint32_t packet_count) override;

private:
    void evaluate() override;

    Analyzer_NotCrashed_Result *_result = NULL;
};

#endif
