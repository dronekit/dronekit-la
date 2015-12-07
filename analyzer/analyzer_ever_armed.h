#ifndef ANALYZER_EVER_ARMED_H
#define ANALYZER_EVER_ARMED_H

/*
 * analyzer_ever_armed
 *
 */

#include "analyzer.h"

class Analyzer_Ever_Armed_Result : public Analyzer_Result_Summary {
public:
    Analyzer_Ever_Armed_Result() :
        Analyzer_Result_Summary()
        { }

    void set_arm_time(uint64_t timestamp) { _arm_time = timestamp; }
    uint64_t arm_time() const { return _arm_time; }

    void to_json(Json::Value &root) const override;

private:
    uint64_t _arm_time = 0;
};

class Analyzer_Ever_Armed : public Analyzer {

public:

    Analyzer_Ever_Armed(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    { }

    bool configure(INIReader *config);

    const std::string name() const override { return "Ever Armed"; }
    const std::string description() const override {
        return "Vehicles typically need to progress through a sequence of arming steps before they can move.  This test will FAIL if the craft did not arm";
    }

    void evaluate() override;
    void end_of_log(const uint32_t packet_count) override;

private:
    Analyzer_Ever_Armed_Result _result;
};

#endif
