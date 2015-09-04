#ifndef ANALYZER_EVER_ARMED_H
#define ANALYZER_EVER_ARMED_H

/*
 * analyzer_ever_armed
 *
 */

#include "analyzer.h"

class Analyzer_Ever_Armed : public Analyzer {

public:
    Analyzer_Ever_Armed(AnalyzerVehicle::Base *&vehicle) :
	Analyzer(vehicle)
    { }

    const char *name() const { return "Ever Armed"; }
    const char *description() const {
        return "This test will FAIL if the craft never armed during the log";
    }

    void evaluate() override;

    void results_json_results(Json::Value &root);
private:
    bool ever_armed = false;
    uint64_t arm_time;
};

#endif
