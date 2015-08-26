#ifndef ANALYZER_BROWNOUT_H
#define ANALYZER_BROWNOUT_H

/*
 * analyzer_brownout
 *
 */

#include "analyzer.h"

class Analyzer_Brownout : public Analyzer {

public:
    Analyzer_Brownout(int fd, struct sockaddr_in &sa, AnalyzerVehicle::Base *&vehicle) :
	Analyzer(fd, sa, vehicle),
        seen_packets(false)
    {
    }

    const char *name() { return "Brownout"; }
    const char *description() {
        return "Log does not end while vehicle appears to be flying";
    }
    bool configure(INIReader *config);

    void results_json_results(Json::Value &root);

private:
    bool seen_packets;

    const double max_last_altitude = 5.0f; // metres
};

#endif


