#ifndef ANALYZER_BROWNOUT_H
#define ANALYZER_BROWNOUT_H

/*
 * analyzer_brownout
 *
 */

#include "analyzer.h"

class Analyzer_Brownout : public Analyzer {

public:
    Analyzer_Brownout(int fd, struct sockaddr_in *sa, AnalyzerVehicle::Base *&vehicle) :
	Analyzer(fd, sa, vehicle)
    {
    }

    const char *name() const { return "Brownout"; }
    const char *description() const {
        return "This test will FAIL if the craft appears to lose onboard power during a flight";
    }
    bool configure(INIReader *config);

    void results_json_results(Json::Value &root);

private:
    bool seen_packets = false;

    const double max_last_altitude = 5.0f; // metres
};

#endif


