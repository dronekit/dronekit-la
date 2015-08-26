#ifndef ANALYZER_EVER_ARMED_H
#define ANALYZER_EVER_ARMED_H

/*
 * analyzer_ever_armed
 *
 */

#include "analyzer.h"

class Analyzer_Ever_Armed : public Analyzer {

public:
    Analyzer_Ever_Armed(int fd, struct sockaddr_in &sa, AnalyzerVehicle::Base *&vehicle) :
	Analyzer(fd, sa, vehicle),
        ever_armed(false)        
    { }

    const char *name() { return "Ever Armed"; }
    const char *description();

    void handle_decoded_message(uint64_t T, mavlink_heartbeat_t &hearbeat) override;

    void results_json_results(Json::Value &root);
private:
    bool ever_armed;
    uint64_t arm_time;
};

#endif
