#ifndef ANALYZER_AUTOPILOT_H
#define ANALYZER_AUTOPILOT_H

/*
 * analyzer_autopilot
 *
 */

#include "analyzer.h"
#include "data_sources.h"


class Analyzer_AutoPilot_Result_Slices_Max : public Analyzer_Result_Period {
public:
    Analyzer_AutoPilot_Result_Slices_Max() :
        _slices_max_max(0)        
        { }

    uint64_t _slices_max_max;
private:
};

class Analyzer_Autopilot : public Analyzer {

public:
    Analyzer_Autopilot(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle, data_sources),
        _result_slices_max(NULL)
    { }

    // check for freemem dropping while we are armed
    // check for scheduling overruns
    // i2c errors
    // autopilot voltages
    // load

    const std::string name() const override { return "AutoPilot Health"; }
    const std::string description() const override {
        return "Many autopilots are capable of monitoring their own performance.  This test will FAIL if problems are detected with the autopilot";
    }

    bool configure(INIReader *config) override;

private:
    const uint16_t dodgy_assumed_slices_count = 2500;
    const uint16_t _slices_max_threshold = 2 * dodgy_assumed_slices_count;

    uint64_t count_overruns;
    uint64_t count_loops;

    void evaluate() override;

    void close_result_slices_max();
    void update_result_slices_max(uint16_t slices);
    void open_result_slices_max(uint16_t slices);

    Analyzer_AutoPilot_Result_Slices_Max *_result_slices_max;

    void end_of_log(uint32_t packet_count) override;
};

#endif


