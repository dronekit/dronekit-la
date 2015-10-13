#ifndef ANALYZE_ATTITUDE_CONTROL_H
#define ANALYZE_ATTITUDE_CONTROL_H

/*
 * analyze_attitudecontrol
 *
 */

#include "analyzer.h"

#include <set>

class Analyzer_Attitude_Control_Result : public Analyzer_Result_Period {
public:
    //FIXME: scope
    float deltamax;
    float roll_at_deltamax;
    float desroll_at_deltamax;
    float pitch_at_deltamax;
    float despitch_at_deltamax;
    bool motors_clipping = false;
    std::set<uint8_t> motors_failing;
    std::set<uint8_t> motors_clipping_high;
    std::set<uint8_t> motors_clipping_low;
};


class Analyzer_Attitude_Control : public Analyzer {

public:

    Analyzer_Attitude_Control(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    { }

    const std::string name() const override { return "Attitude Control"; }
    const std::string description() const override {
        return "This test will FAIL if the craft's desired attitudes and achieved attitudes do not match for more than a threshold time";
    }

    bool configure(INIReader *config) override;

    void evaluate() override;

private:

    float offset_warn = 5.0f;
    float offset_fail = 10.0f;
    uint32_t duration_min = 250000; // microseconds

    void end_of_log(uint32_t packet_count);

    void do_add_evilness(struct compass_offset_result result);

    Analyzer_Attitude_Control_Result *_result = NULL;
    void open_result(double delta);
    void update_result(double delta);
    void close_result();
};

#endif
