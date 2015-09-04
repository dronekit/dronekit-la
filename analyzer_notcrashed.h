#ifndef ANALYZE_NOTCRASHED_H
#define ANALYZE_NOTCRASHED_H

/*
 * analyze_notcrashed
 *
 */

#include "analyzer.h"

#define angle_max_default_degrees 20

class Analyzer_NotCrashed : public Analyzer {

public:
    Analyzer_NotCrashed(AnalyzerVehicle::Base *&vehicle) :
	Analyzer(vehicle)
    { }

    const uint16_t servo_output_threshold = 1250;
    
    const char *name() const override { return "Crash Test"; }
    const char *description() const override {
        return "This test will FAIL if the vehicle appears to crash";
    }

    void end_of_log(const uint32_t packet_count);

    class notcrashed_result : public analyzer_result {
    public:
        uint64_t timestamp_start;
        uint64_t timestamp_stop;
        float angle;
        float angle_max;
        double servo_output[17]; // FIXME take 17 from somewhere...
        uint64_t duration() { return (timestamp_stop - timestamp_start); }
    };
    #define MAX_NOTCRASHED_RESULTS 100
    uint8_t notcrashed_results_offset = 0;
    notcrashed_result notcrashed_results[MAX_NOTCRASHED_RESULTS];
    bool notcrashed_results_overrun;

private:
    void evaluate() override;

    void add_series(Json::Value &root);
        
    // const double angle_max_default_degrees = 20.0f;
    
    void results_json_results(Json::Value &root);
};

#endif
